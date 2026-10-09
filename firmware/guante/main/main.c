// Guante de teleoperación - ESP32 + MPU-6050
// Mide la inclinación de la mano (cabeceo y alabeo) con un filtro complementario
// (giroscopio + acelerómetro) y un IIR de primer orden, y la envía por UDP broadcast
// al PC. El nodo glove_teleop de ROS2 la convierte en /cmd_vel para el robot.
//
//   Mano inclinada hacia adelante / atrás  -> avanzar / retroceder
//   Mano inclinada a la izquierda / derecha -> girar
//
// Paquete (texto, 50 por segundo):
//   GL,t_ms,pitch,roll,pitch_raw,roll_raw
//   pitch y roll: grados respecto a la posición neutra, filtrados
//   *_raw: grados respecto a la posición neutra, solo acelerómetro (sin filtrar)
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/sockets.h"

#include "config.h"
#include "mpu6050.h"

static const char *TAG = "guante";

#define RAD2DEG (180.0f / (float)M_PI)

static volatile bool s_connected;
static volatile bool s_rezero;

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        gpio_set_level(PIN_LED, 0);
        ESP_LOGW(TAG, "WiFi desconectado, reintentando...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_connected = true;
        gpio_set_level(PIN_LED, 1);
        ESP_LOGI(TAG, "Conectado a %s, IP del guante: " IPSTR " (envía a UDP %d)",
                 WIFI_SSID, IP2STR(&ev->ip_info.ip), GLOVE_UDP_PORT);
    }
}

static void wifi_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event, NULL, NULL));

    wifi_config_t wc = {0};
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   // sin ahorro de energía: menor latencia
}

// ---------------------------------------------------------------------------
// Botón BOOT: fija la posición actual de la mano como neutra
// ---------------------------------------------------------------------------
static void button_task(void *arg)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << PIN_BUTTON,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io);
    int prev = 1;
    while (1) {
        int now = gpio_get_level(PIN_BUTTON);
        if (prev == 1 && now == 0) s_rezero = true;   // flanco de bajada = presionado
        prev = now;
        vTaskDelay(pdMS_TO_TICKS(30));                // también sirve de antirrebote
    }
}

// ---------------------------------------------------------------------------
// Ángulos
// ---------------------------------------------------------------------------
// Ángulos solo con el acelerómetro (grados): válidos con la mano quieta, ruidosos al moverla
static void accel_angles(const imu_sample_t *s, float *pitch, float *roll)
{
    *pitch = atan2f(-s->ax, sqrtf(s->ay * s->ay + s->az * s->az)) * RAD2DEG;
    *roll = atan2f(s->ay, s->az) * RAD2DEG;
}

// Promedia la posición de la mano durante n muestras para usarla como cero
static void measure_neutral(float *p0, float *r0)
{
    float sp = 0, sr = 0;
    int ok = 0;
    for (int i = 0; i < NEUTRAL_SAMPLES; i++) {
        imu_sample_t s;
        if (mpu_read(&s) == ESP_OK) {
            float p, r;
            accel_angles(&s, &p, &r);
            sp += p;
            sr += r;
            ok++;
        }
        vTaskDelay(pdMS_TO_TICKS(1000 / SAMPLE_HZ));
    }
    *p0 = ok ? sp / ok : 0.0f;
    *r0 = ok ? sr / ok : 0.0f;
    ESP_LOGI(TAG, "Posición neutra: cabeceo %.1f°, alabeo %.1f°", *p0, *r0);
}

static void glove_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    struct sockaddr_in dst = {
        .sin_family = AF_INET,
        .sin_port = htons(GLOVE_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };

    float p0, r0;
    measure_neutral(&p0, &r0);

    // Estado del filtro complementario (ángulo absoluto) y del IIR (respecto al neutro)
    imu_sample_t s = {0};
    mpu_read(&s);
    float pitch_c, roll_c;
    accel_angles(&s, &pitch_c, &roll_c);
    float pitch_f = pitch_c - p0, roll_f = roll_c - r0;

    const float dt = 1.0f / SAMPLE_HZ;
    TickType_t last = xTaskGetTickCount();
    int n = 0;
    uint32_t errors = 0;
    int64_t last_log = 0;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / SAMPLE_HZ));

        if (s_rezero) {
            s_rezero = false;
            ESP_LOGI(TAG, "Botón: tomando nueva posición neutra, mantén la mano quieta");
            measure_neutral(&p0, &r0);
            pitch_c = p0;
            roll_c = r0;
            pitch_f = roll_f = 0.0f;
            last = xTaskGetTickCount();
            continue;
        }

        if (mpu_read(&s) != ESP_OK) {
            errors++;
            continue;
        }

        // Filtro complementario: el giroscopio sigue los movimientos rápidos y el
        // acelerómetro corrige la deriva lenta. gx gira alrededor de X (alabeo),
        // gy alrededor de Y (cabeceo).
        float pa, ra;
        accel_angles(&s, &pa, &ra);
        pitch_c = COMP_ALPHA * (pitch_c + s.gy * dt) + (1.0f - COMP_ALPHA) * pa;
        roll_c = COMP_ALPHA * (roll_c + s.gx * dt) + (1.0f - COMP_ALPHA) * ra;

        // IIR de primer orden sobre el ángulo respecto al neutro (quita el temblor de la mano)
        pitch_f += IIR_ALPHA * ((pitch_c - p0) - pitch_f);
        roll_f += IIR_ALPHA * ((roll_c - r0) - roll_f);

        if (++n >= SEND_DIV) {
            n = 0;
            if (s_connected) {
                char msg[96];
                int len = snprintf(msg, sizeof(msg), "GL,%lld,%.2f,%.2f,%.2f,%.2f",
                                   esp_timer_get_time() / 1000, pitch_f, roll_f, pa - p0, ra - r0);
                sendto(sock, msg, len, 0, (struct sockaddr *)&dst, sizeof(dst));
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_log > 1000000) {
            last_log = now;
            ESP_LOGI(TAG, "WiFi %s | cabeceo %6.1f° | alabeo %6.1f° | errores IMU %lu",
                     s_connected ? "ok" : "no", pitch_f, roll_f, (unsigned long)errors);
        }
    }
}

void app_main(void)
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);

    ESP_LOGI(TAG, "Guante arrancando: deja la mano quieta y horizontal");
    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "Sin IMU: revisa el cableado y reinicia");
        while (1) {   // parpadeo rápido = error de IMU
            gpio_set_level(PIN_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(PIN_LED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    mpu_calibrate_gyro(GYRO_CAL_SAMPLES);

    wifi_init();
    xTaskCreate(button_task, "boton", 2048, NULL, 3, NULL);
    xTaskCreate(glove_task, "guante", 4096, NULL, 5, NULL);
}
