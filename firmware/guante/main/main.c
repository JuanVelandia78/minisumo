// Guante de teleoperación - ESP32 DevKit + MPU-6050
// Mide la inclinación de la mano (cabeceo y alabeo) con un filtro complementario
// (giroscopio + acelerómetro) y un IIR de primer orden, y la envía por UDP al PC (broadcast
// hasta que glove_teleop avisa con "GLP"; luego directo). glove_teleop la convierte en /cmd_vel.
// Además filtra la IMU cruda con un IIR Butterworth (orden 2, fc 10 Hz) y envía crudo y filtrado.
//
//   Mano inclinada hacia adelante / atrás  -> avanzar / retroceder
//   Mano inclinada a la izquierda / derecha -> girar
//
// Paquete (texto, 50 por segundo):
//   GL,t_ms,pitch,roll,pitch_raw,roll_raw,ax,ay,az,gx,gy,gz,ax_f,ay_f,az_f,gx_f,gy_f,gz_f
//   pitch y roll: grados respecto a la posición neutra, filtrados
//   *_raw: grados respecto a la posición neutra, solo acelerómetro (sin filtrar)
//   ax..gz: IMU cruda (g, °/s sin bias); *_f: la misma después del IIR
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
#include "soc/soc_caps.h"
#if SOC_TEMP_SENSOR_SUPPORTED
#include "driver/temperature_sensor.h"
#endif
#include "lwip/sockets.h"

#include "config.h"
#include "mpu6050.h"

static const char *TAG = "guante";

#define RAD2DEG (180.0f / (float)M_PI)

static volatile bool s_connected;
static volatile bool s_rezero;
#if SOC_TEMP_SENSOR_SUPPORTED
static temperature_sensor_handle_t s_tsens;   // sensor interno del chip (NULL si no arrancó)
#endif

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        gpio_set_level(PIN_LED, LED_OFF);
        ESP_LOGW(TAG, "WiFi desconectado, reintentando...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        s_connected = true;
        gpio_set_level(PIN_LED, LED_ON);
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
    esp_wifi_set_ps(WIFI_PS_NONE);   // sin ahorro de energía: recibe al instante el aviso del PC
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
// Lleva un ángulo al rango [-180, 180) grados
// Biquad (forma directa II transpuesta) para el IIR de la IMU cruda
typedef struct { float z1, z2; } biquad_t;

static float biquad(biquad_t *f, float x)
{
    float y = IMU_IIR_B0 * x + f->z1;
    f->z1 = IMU_IIR_B1 * x - IMU_IIR_A1 * y + f->z2;
    f->z2 = IMU_IIR_B2 * x - IMU_IIR_A2 * y;
    return y;
}

static float wrap180(float a)
{
    while (a >= 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

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
    // Escucha en el mismo puerto: glove_teleop envía "GLP" cada 0,5 s y desde ahí el guante le
    // envía directo (unicast). El broadcast solo se usa mientras no hay PC: los hotspots de
    // celular retienen o descartan los broadcast y llegaban 6-14 de los 50 paquetes por segundo.
    struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(GLOVE_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&local, sizeof(local));
    fcntl(sock, F_SETFL, O_NONBLOCK);
    struct sockaddr_in bcast = {
        .sin_family = AF_INET,
        .sin_port = htons(GLOVE_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    struct sockaddr_in pc = {0};
    int64_t pc_seen_us = 0;

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
    static biquad_t imu_f[6];           // ax, ay, az, gx, gy, gz
    float imf[6] = {0};
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

        // IIR pasa-bajos sobre la IMU cruda (para comparar crudo y filtrado en el PC)
        const float raw[6] = {s.ax, s.ay, s.az, s.gx, s.gy, s.gz};
        for (int k = 0; k < 6; k++) imf[k] = biquad(&imu_f[k], raw[k]);

        // Filtro complementario: el giroscopio sigue los movimientos rápidos y el
        // acelerómetro corrige la deriva lenta. gx gira alrededor de X (alabeo),
        // gy alrededor de Y (cabeceo).
        float pa, ra;
        accel_angles(&s, &pa, &ra);
        // La corrección se calcula con la diferencia envuelta a ±180°: el alabeo del acelerómetro
        // salta de +180 a -180 al voltear la mano, y sin esto el filtro promediaba los dos extremos.
        float pitch_g = pitch_c + s.gy * dt, roll_g = roll_c + s.gx * dt;
        pitch_c = wrap180(pitch_g + (1.0f - COMP_ALPHA) * wrap180(pa - pitch_g));
        roll_c = wrap180(roll_g + (1.0f - COMP_ALPHA) * wrap180(ra - roll_g));

        // IIR de primer orden sobre el ángulo respecto al neutro (quita el temblor de la mano)
        pitch_f = wrap180(pitch_f + IIR_ALPHA * wrap180(wrap180(pitch_c - p0) - pitch_f));
        roll_f = wrap180(roll_f + IIR_ALPHA * wrap180(wrap180(roll_c - r0) - roll_f));

        // Aviso del PC: guarda su dirección para enviarle directo
        char rx[16];
        struct sockaddr_in src;
        socklen_t sl = sizeof(src);
        while (recvfrom(sock, rx, sizeof(rx), 0, (struct sockaddr *)&src, &sl) >= 3) {
            if (rx[0] == 'G' && rx[1] == 'L' && rx[2] == 'P') {
                if (pc.sin_addr.s_addr != src.sin_addr.s_addr) {
                    ESP_LOGI(TAG, "PC en " IPSTR ": envío directo", IP2STR((esp_ip4_addr_t *)&src.sin_addr));
                }
                pc = src;
                pc_seen_us = esp_timer_get_time();
            }
            sl = sizeof(src);
        }
        bool pc_ok = pc_seen_us && esp_timer_get_time() - pc_seen_us < PC_TIMEOUT_US;

        if (++n >= SEND_DIV) {
            n = 0;
            if (s_connected) {
                // GL,t,pitch,roll,pitch_raw,roll_raw, ax,ay,az,gx,gy,gz (crudos), ax_f..gz_f (filtrados)
                char msg[256];
                int len = snprintf(msg, sizeof(msg),
                                   "GL,%lld,%.2f,%.2f,%.2f,%.2f,"
                                   "%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f",
                                   esp_timer_get_time() / 1000, pitch_f, roll_f, wrap180(pa - p0), wrap180(ra - r0),
                                   s.ax, s.ay, s.az, s.gx, s.gy, s.gz,
                                   imf[0], imf[1], imf[2], imf[3], imf[4], imf[5]);
                struct sockaddr_in *dst = pc_ok ? &pc : &bcast;
                sendto(sock, msg, len, 0, (struct sockaddr *)dst, sizeof(*dst));
            }
        }

        int64_t now = esp_timer_get_time();
        if (now - last_log > 1000000 / LOG_HZ) {
            last_log = now;
            float chip_c = NAN;   // la ESP32 clásica no tiene sensor de temperatura utilizable: sale "nan"
#if SOC_TEMP_SENSOR_SUPPORTED
            if (s_tsens) temperature_sensor_get_celsius(s_tsens, &chip_c);
#endif
            // abs: ángulo respecto a la horizontal (filtro complementario)
            // rel: respecto a la posición neutra, filtrado (lo que se envía al PC)
            // acel: solo acelerómetro, sin filtrar (para comparar)
            ESP_LOGI(TAG, "cab abs %6.1f rel %6.1f acel %6.1f | alab abs %6.1f rel %6.1f acel %6.1f | "
                          "T MPU %4.1f C, chip %4.1f C | WiFi %s | PC %s | err IMU %lu",
                     pitch_c, pitch_f, wrap180(pa - p0), roll_c, roll_f, wrap180(ra - r0),
                     s.temp, chip_c, s_connected ? "ok" : "no", pc_ok ? "directo" : "broadcast", (unsigned long)errors);
        }
    }
}

// Sensor de temperatura interno del chip (solo en los modelos que lo tienen, como la C3)
static void tsens_init(void)
{
#if SOC_TEMP_SENSOR_SUPPORTED
    temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(-10, 80);
    if (temperature_sensor_install(&cfg, &s_tsens) != ESP_OK || temperature_sensor_enable(s_tsens) != ESP_OK) {
        ESP_LOGW(TAG, "Sensor de temperatura del chip no disponible");
        s_tsens = NULL;
    }
#else
    ESP_LOGW(TAG, "Este chip no tiene sensor de temperatura interno: solo se muestra la del MPU");
#endif
}

void app_main(void)
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, LED_OFF);

    ESP_LOGI(TAG, "Guante arrancando: deja la mano quieta y horizontal");
    if (mpu_init() != ESP_OK) {
        ESP_LOGE(TAG, "Sin IMU: revisa el cableado y reinicia");
        while (1) {   // parpadeo rápido = error de IMU
            gpio_set_level(PIN_LED, LED_ON);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(PIN_LED, LED_OFF);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    mpu_calibrate_gyro(GYRO_CAL_SAMPLES);
    tsens_init();

    wifi_init();
    xTaskCreate(button_task, "boton", 2048, NULL, 3, NULL);
    xTaskCreate(glove_task, "guante", 4096, NULL, 5, NULL);
}
