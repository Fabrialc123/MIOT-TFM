#include "uart_app.h"


static const uint16_t RX_BUF_SIZE = CONFIG_UART_APP_RX_BUFFER_SIZE;

void uart_app_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = CONFIG_UART_APP_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1,CONFIG_UART_APP_TXD, CONFIG_UART_APP_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

int16_t uart_app_sendData(const char* logName, const uint8_t* data, uint32_t len)
{
    const int16_t txBytes = uart_write_bytes(UART_NUM_1, data, len);
    ESP_LOGI(logName, "uart_app_sendData: Wrote: %.*s", txBytes, data);
    return txBytes;
}




