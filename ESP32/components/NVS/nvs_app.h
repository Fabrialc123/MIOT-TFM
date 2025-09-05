#ifndef MAIN_NVS_APP_H_
#define MAIN_NVS_APP_H_

#include <stdint.h>
#include <stddef.h>
#include <esp_err.h>


void nvs_app_init();

esp_err_t nvs_app_get_uint8_value(const char *key, uint8_t *value);
void nvs_app_set_uint8_value(const char *key, const uint8_t value);

esp_err_t nvs_app_get_uint16_value(const char *key, uint16_t *value);
void nvs_app_set_uint16_value(const char *key, const uint16_t value);

esp_err_t nvs_app_get_uint32_value(const char *key, uint32_t *value);
void nvs_app_set_uint32_value(const char *key, const uint32_t value);

esp_err_t nvs_app_get_string_value(const char *key, char *value, size_t *size);
void nvs_app_set_string_value(const char *key, const char *value);

esp_err_t nvs_app_get_blob_value(const char *key, void *value, size_t *size);
void nvs_app_set_blob_value(const char *key, const void *value, const size_t size);



#endif /* MAIN_NVS_APP_H_ */
