#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_event.h"
#include "esp_wifi.h"

ESP_EVENT_DECLARE_BASE(WIFI_MANAGER_EVENTS);

typedef enum { WIFI_DISCONNECTED, WIFI_CONNECTED } wifi_manager_events_e;

void wifi_manager_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);
void wifi_manager_init_sta();
void wifi_manager_init_softap();
void wifi_manager_init_apsta();
void wifi_manager_disconnect();

void wifi_manager_get_ip_address(char *ip);

#endif // WIFI_MANAGER_H
