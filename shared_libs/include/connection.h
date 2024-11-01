/*
It is libary for easier use of WIFI

Author: https://github.com/SebastianL-NT
*/
#ifndef CONNECTION_H
#define CONNECTION_H

// Includes
#include "settings.h"
#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_err.h"

// Public functions
esp_err_t wifiInit();
esp_err_t mqttClientStart();
void mqttPublish(char *topic, char *data);

extern esp_err_t mqtt_ret;

#endif
