#ifndef COMPONENTS_UART_APP_UART_APP_H_
#define COMPONENTS_UART_APP_UART_APP_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"

void uart_app_init(void);
int16_t uart_app_sendData(const char* logName, const uint8_t* data, uint32_t len);

#endif /* COMPONENTS_UART_APP_UART_APP_H_ */
