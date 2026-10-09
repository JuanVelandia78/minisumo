// WiFi (estación) y comunicación UDP con el PC
#pragma once
#include <stdbool.h>

typedef void (*net_cmd_cb_t)(const char *msg, int len);

// Conecta a WIFI_SSID y arranca la tarea de recepción. cb se llama por cada datagrama recibido.
void net_start(net_cmd_cb_t cb);
// Envía al PC (la dirección del último comando recibido). false si aún no hay PC conocido.
bool net_send(const char *buf, int len);
bool net_connected(void);
bool net_pc_known(void);
const char *net_ip_str(void);
