#include "wifi_manager.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include <stdint.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#define EXAMPLE_ESP_WIFI_SSID_STA CONFIG_ESP_WIFI_SSID_STA
#define EXAMPLE_ESP_WIFI_PASS_STA CONFIG_ESP_WIFI_PASSWORD_STA
#define EXAMPLE_ESP_MAXIMUM_RETRY_STA CONFIG_ESP_MAXIMUM_RETRY_STA

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE_STA WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER_STA ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE_STA WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER_STA CONFIG_ESP_WIFI_PW_ID_STA
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE_STA WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER_STA CONFIG_ESP_WIFI_PW_ID_STA
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA WIFI_AUTH_WAPI_PSK
#endif

#define EXAMPLE_ESP_WIFI_SSID_AP CONFIG_ESP_WIFI_SSID_AP
#define EXAMPLE_ESP_WIFI_PASS_AP CONFIG_ESP_WIFI_PASSWORD_AP
#define EXAMPLE_ESP_WIFI_CHANNEL_AP CONFIG_ESP_WIFI_CHANNEL_AP
#define EXAMPLE_MAX_STA_CONN_AP CONFIG_ESP_MAX_STA_CONN_AP

static const char *TAG = "Wi-Fi Manager";
EventGroupHandle_t wifi_manager_event_group;
ESP_EVENT_DEFINE_BASE(WIFI_MANAGER_EVENTS);

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

int retry_num = 0;

char ip_address[16] = "N/A";

// Manejador de eventos
void wifi_manager_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_STA_START:
      esp_wifi_connect();
      break;
    case WIFI_EVENT_STA_DISCONNECTED:
      ESP_LOGW(TAG, "WIFI_EVENT_STA_DISCONNECTED, REASON: %d",
               ((wifi_event_sta_disconnected_t *)event_data)->reason);
      if (retry_num < EXAMPLE_ESP_MAXIMUM_RETRY_STA) {
        esp_wifi_connect();
        retry_num++;
        ESP_LOGI(TAG, "retry to connect to the AP");
      } else {
        xEventGroupSetBits(wifi_manager_event_group, WIFI_FAIL_BIT);
      }
      ESP_LOGI(TAG, "connect to the AP fail");
      break;

    case WIFI_EVENT_STA_STOP:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_STOP");
      break;

    case WIFI_EVENT_STA_CONNECTED:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_CONNECTED");
      break;

    case WIFI_EVENT_STA_AUTHMODE_CHANGE:
      ESP_LOGI(TAG, "WIFI_EVENT_STA_AUTHMODE_CHANGE");
      break;

    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "WIFI_EVENT_AP_START");
      break;

    case WIFI_EVENT_AP_STOP:
      ESP_LOGI(TAG, "WIFI_EVENT_AP_STOP");
      break;

    case WIFI_EVENT_AP_PROBEREQRECVED:
      ESP_LOGI(TAG, "WIFI_EVENT_AP_PROBEREQRECVED");
      break;

    case WIFI_EVENT_AP_STACONNECTED:
      wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
      ESP_LOGI(TAG,
               "WIFI_EVENT_AP_STACONNECTED, station " MACSTR " join, AID=%d",
               MAC2STR(event->mac), event->aid);
      break;

    case WIFI_EVENT_AP_STADISCONNECTED:
      wifi_event_ap_stadisconnected_t *ev =
          (wifi_event_ap_stadisconnected_t *)event_data;
      ESP_LOGI(TAG,
               "WIFI_EVENT_AP_STACONNECTED, station " MACSTR " leave, AID=%d",
               MAC2STR(ev->mac), ev->aid);
      break;

    default:
      ESP_LOGW(TAG, "UNKNOWN WIFI_EVENT (%ld)", event_id);
      break;
    }
  }

  if (event_base == IP_EVENT) {
    switch (event_id) {
    case IP_EVENT_STA_GOT_IP:
      sprintf(ip_address, IPSTR,
              IP2STR(&((ip_event_got_ip_t *)event_data)->ip_info.ip));
      // ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&((ip_event_got_ip_t*)
      // event_data)->ip_info.ip));
      ESP_LOGI(TAG, "got ip: %s", ip_address);
      retry_num = 0;
      xEventGroupSetBits(wifi_manager_event_group, WIFI_CONNECTED_BIT);
      esp_event_post(WIFI_MANAGER_EVENTS, WIFI_CONNECTED, NULL, 0, 100);
      break;

    case IP_EVENT_STA_LOST_IP:
      ESP_LOGW(TAG, "IP_EVENT_STA_LOST_IP");
      esp_event_post(WIFI_MANAGER_EVENTS, WIFI_DISCONNECTED, NULL, 0, 100);
      break;

    default:
      ESP_LOGW(TAG, "UNKNOWN IP_EVENT (%ld)", event_id);
      break;
    }
  }
}

void wifi_manager_init_sta() {

  wifi_manager_event_group = xEventGroupCreate();

  // ESP_ERROR_CHECK(esp_netif_init());
  // esp_netif_create_default_wifi_sta();

  // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_event_handler, NULL, NULL));

  wifi_config_t wifi_config;
  esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
  if (strlen((char*)wifi_config.sta.ssid) != 0) {
    ESP_LOGI(TAG, "Taking credentials from nvs");
    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = CONFIG_ESP_WIFI_SSID_STA,
                .password = CONFIG_ESP_WIFI_PASSWORD_STA,
                .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA,
                .sae_pwe_h2e = ESP_WIFI_SAE_MODE_STA,
                .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER_STA,
            },
    };
  } else {
    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = CONFIG_ESP_WIFI_SSID_STA,
                .password = CONFIG_ESP_WIFI_PASSWORD_STA,
                .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA,
                .sae_pwe_h2e = ESP_WIFI_SAE_MODE_STA,
                .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER_STA,
            },
    };
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Station started");

  EventBits_t bits = xEventGroupWaitBits(wifi_manager_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s", EXAMPLE_ESP_WIFI_SSID_STA);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID_STA, EXAMPLE_ESP_WIFI_PASS_STA);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

void wifi_manager_init_softap() {

  wifi_manager_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = EXAMPLE_ESP_WIFI_SSID_AP,
              .password = EXAMPLE_ESP_WIFI_PASS_AP,
              .channel = EXAMPLE_ESP_WIFI_CHANNEL_AP,
              .max_connection = EXAMPLE_MAX_STA_CONN_AP,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
              .authmode = WIFI_AUTH_WPA3_PSK,
              .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else
              .authmode = WIFI_AUTH_WPA2_PSK,
#endif /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
              .pmf_cfg =
                  {
                      .required = true,
                  },
          },
  };
  if (strlen(EXAMPLE_ESP_WIFI_PASS_AP) == 0) {
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "SoftAP started, SSID:%s", EXAMPLE_ESP_WIFI_SSID_AP);
}

void wifi_manager_init_apsta() {

  wifi_manager_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  esp_netif_create_default_wifi_sta();
  esp_netif_create_default_wifi_ap();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, ESP_EVENT_ANY_ID, &wifi_manager_event_handler, NULL, NULL));

  wifi_config_t wifi_sta_config = {
      .sta =
          {
              .ssid = CONFIG_ESP_WIFI_SSID_STA,
              .password = CONFIG_ESP_WIFI_PASSWORD_STA,
              .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD_STA,
              .sae_pwe_h2e = ESP_WIFI_SAE_MODE_STA,
              .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER_STA,
          },
  };
  wifi_config_t wifi_ap_config = {
      .ap =
          {
              .ssid = EXAMPLE_ESP_WIFI_SSID_AP,
              .password = EXAMPLE_ESP_WIFI_PASS_AP,
              .channel = EXAMPLE_ESP_WIFI_CHANNEL_AP,
              .max_connection = EXAMPLE_MAX_STA_CONN_AP,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
              .authmode = WIFI_AUTH_WPA3_PSK,
              .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else
              .authmode = WIFI_AUTH_WPA2_PSK,
#endif /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
              .pmf_cfg =
                  {
                      .required = true,
                  },
          },
  };
  if (strlen(EXAMPLE_ESP_WIFI_PASS_AP) == 0) {
    wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));

  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "APSTA started");

  EventBits_t bits = xEventGroupWaitBits(wifi_manager_event_group,
                                         WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                         pdFALSE, pdFALSE, portMAX_DELAY);

  if (bits & WIFI_CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s", EXAMPLE_ESP_WIFI_SSID_STA);
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
             EXAMPLE_ESP_WIFI_SSID_STA, EXAMPLE_ESP_WIFI_PASS_STA);
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

void wifi_manager_disconnect() {
  ESP_ERROR_CHECK(esp_wifi_stop());
  ESP_ERROR_CHECK(esp_wifi_deinit());
}

void wifi_manager_get_ip_address(char *ip) { strcpy(ip, ip_address); }
