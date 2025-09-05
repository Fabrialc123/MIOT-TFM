#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_timer.h"

typedef struct {
    uint64_t wakeup_timer;
    void *timer_handler;
    void *gpio_handler;
} light_sleep_params_t;

void light_sleep_task(void *pvParameters) {
    light_sleep_params_t *params = (light_sleep_params_t *)pvParameters;

    uint64_t wakeup_timer = params->wakeup_timer;
    void (*timer_handler)() = params->timer_handler;
    void (*gpio_handler)() = params->gpio_handler;

	int64_t next_sleep_duration = wakeup_timer;

    while (true) {
        ESP_LOGD(pcTaskGetName(0),"Entering light sleep");

        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(next_sleep_duration));

        int64_t time_before_sleep = esp_timer_get_time();

        esp_light_sleep_start();

        int64_t time_after_wakeup = esp_timer_get_time();
        int64_t time_slept = time_after_wakeup - time_before_sleep;

        const char* wakeup_reason;
        switch (esp_sleep_get_wakeup_cause()) {
            case ESP_SLEEP_WAKEUP_TIMER:
                wakeup_reason = "timer";
                timer_handler(1);
                next_sleep_duration = wakeup_timer;
                break;
            case ESP_SLEEP_WAKEUP_EXT0:
                wakeup_reason = "pin";
                gpio_handler(1);
                next_sleep_duration -= time_slept;

                if (next_sleep_duration < 0) {
                    next_sleep_duration = 0;
                }
                break;
            default:
                wakeup_reason = "UNKNOWN";
                break;
        }
        ESP_LOGD(pcTaskGetName(0),"Returned from light sleep, reason: %s, slept for %lld ms",
                wakeup_reason, time_slept / 1000);
    }
    vTaskDelete(NULL);
}
