#include <string.h>
#include <stdio.h>
#include "net.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"

static const char *TAG = "net";

static volatile bool s_connected;
static char s_ip[16] = "0.0.0.0";
static net_cmd_cb_t s_cb;

static int s_tx_sock = -1;
static struct sockaddr_in s_pc;
static volatile bool s_pc_known;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

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
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&ev->ip_info.ip));
        s_connected = true;
        gpio_set_level(PIN_LED, 1);
        ESP_LOGI(TAG, "Conectado a %s, IP del robot: %s (comandos en UDP %d)", WIFI_SSID, s_ip, UDP_CMD_PORT);
    }
}

static void wifi_init(void)
{
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_LED, 0);
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
    esp_wifi_set_max_tx_power(WIFI_MAX_TX_POWER);
    esp_wifi_set_ps(WIFI_PS_NONE);                  // sin ahorro de energía: menor latencia
}

static void rx_task(void *arg)
{
    int sock;
    while ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0) vTaskDelay(pdMS_TO_TICKS(500));

    struct sockaddr_in local = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_CMD_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    bind(sock, (struct sockaddr *)&local, sizeof(local));
    struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in bcast = {
        .sin_family = AF_INET,
        .sin_port = htons(UDP_BEACON_PORT),
        .sin_addr.s_addr = htonl(INADDR_BROADCAST),
    };
    int64_t last_beacon = 0;
    char buf[256];

    while (1) {
        struct sockaddr_in src;
        socklen_t sl = sizeof(src);
        int n = recvfrom(sock, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&src, &sl);
        if (n > 0) {
            buf[n] = '\0';
            portENTER_CRITICAL(&s_mux);
            s_pc = src;                     // la telemetría vuelve a quien envió el último comando
            s_pc_known = true;
            portEXIT_CRITICAL(&s_mux);
            if (s_cb) s_cb(buf, n);
        }

        // Anuncio para que el PC encuentre la IP del robot sin configurarla
        int64_t now = esp_timer_get_time();
        if (s_connected && now - last_beacon > 1000000) {
            last_beacon = now;
            char msg[64];
            int len = snprintf(msg, sizeof(msg), "MINISUMO,%s,%d", s_ip, UDP_CMD_PORT);
            sendto(s_tx_sock, msg, len, 0, (struct sockaddr *)&bcast, sizeof(bcast));
        }
    }
}

void net_start(net_cmd_cb_t cb)
{
    s_cb = cb;
    wifi_init();
    s_tx_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int yes = 1;
    setsockopt(s_tx_sock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
    xTaskCreate(rx_task, "net_rx", 4096, NULL, 5, NULL);
}

bool net_send(const char *buf, int len)
{
    if (!s_pc_known || !s_connected) return false;
    struct sockaddr_in dst;
    portENTER_CRITICAL(&s_mux);
    dst = s_pc;
    portEXIT_CRITICAL(&s_mux);
    return sendto(s_tx_sock, buf, len, 0, (struct sockaddr *)&dst, sizeof(dst)) == len;
}

bool net_connected(void) { return s_connected; }
bool net_pc_known(void) { return s_pc_known; }
const char *net_ip_str(void) { return s_ip; }
