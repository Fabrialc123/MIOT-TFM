#ifndef RH_MAIN_C
#define RH_MAIN_C
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "ttn.h"
#include "Davis-RFM69.h"
#include "nvs_app.h"
#include "mean_sender.c"
#include "light_sleep_mode.c"
#include "wifi_manager.h"
#include "http_server.h"
#include "uart_app.h"

#include "main.h"

#define TAG "MAIN"

#define SEND_INTERVAL_DEFAULT CONFIG_LoRaWAN_SEND_INTERVAL
#define NVS_SEND_INTERVAL_KEY "main_sendInter"
static uint16_t nvs_SEND_INTERVAL;

#define APPEUI_DEFAULT CONFIG_LoRaWAN_AppEUI
#define NVS_APPEUI_KEY "main_appEui"
 char nvs_APPEUI[17];

#define DEVEUI_DEFAULT CONFIG_LoRaWAN_DevEUI
#define NVS_DEVEUI_KEY "main_devEui"
 char nvs_DEVEUI[17];

#define APPKEY_DEFAULT CONFIG_LoRaWAN_AppKey
#define NVS_APPKEY_KEY "main_appKey"
 char nvs_APPKEY[33];

const static bool NVS_LISTEN_STATIONS_DEFAULT[8] = {1, 0, 0, 0, 0, 0, 0, 0};
#define NVS_LISTEN_STATIONS_KEY "main_stations"
static bool nvs_LISTEN_STATIONS[8];

#define DAVIS_DEBUG_DEFAULT 0
#define NVS_DAVIS_DEBUG_KEY "main_davisDebug"
static uint8_t nvs_DAVIS_DEBUG;

static uint8_t lorawan_status = 0;
static uint8_t davis_status = 0;

static bool ENTER_SLEEP_MODE = false;

// Pins and other resources
// SPI Stuff
#if CONFIG_HSPI_HOST
#define TTN_SPI_HOST HSPI_HOST
#elif CONFIG_VSPI_HOST
#define TTN_SPI_HOST      VSPI_HOST	//VSPI_HOST
#endif

#define TTN_SPI_DMA_CHAN  SPI_DMA_DISABLED
#define TTN_PIN_SPI_SCLK  CONFIG_LoRaWAN_SCK_GPIO 	//18
#define TTN_PIN_SPI_MOSI  CONFIG_LoRaWAN_MOSI_GPIO	//23
#define TTN_PIN_SPI_MISO  CONFIG_LoRaWAN_MISO_GPIO	//19
#define TTN_PIN_NSS       CONFIG_LoRaWAN_NSS_GPIO	//5
#define TTN_PIN_RXTX      TTN_NOT_CONNECTED
#define TTN_PIN_RST       CONFIG_LoRaWAN_RST_GPIO	//16
#define TTN_PIN_DIO0      CONFIG_LoRaWAN_DIO0		//26
#define TTN_PIN_DIO1      CONFIG_LoRaWAN_DIO1		//35

#define TX_INTERVAL 30

void messageReceived(const uint8_t* message, size_t length, ttn_port_t port)
{
    printf("Message of %d bytes received on port %d:", length, port);
    for (int i = 0; i < length; i++)
        printf(" %02x", message[i]);
    printf("\n");
}

void rx_task(void *pvParameter)
{
	ESP_LOGI(pcTaskGetName(0), "Start");

	uint8_t len, currChannel;
	t_DavisData davis_data;

	len = DavisRFM69_maxMessageLength();

	if (DavisRFM69_available()) {
		// Should be a message for us now
		uint8_t buf[len];
		char aux[len*3 + 1];

		if (DavisRFM69_recv(buf)) {

			currChannel = DavisRFM69_channel();

			for (size_t i = 0; i < len; i++) {
				sprintf(aux + (i*3), "%02hhX ", buf[i]); // Convierte cada byte a su representación hexadecimal
			}
			//ESP_LOGI(pcTaskGetName(0), "Received:%s",aux);
			//ESP_LOGI(pcTaskGetName(0), "Channel:%d",currChannel);
			//ESP_LOGI(pcTaskGetName(0), "RSSI: %d", DavisRFM69_lastRssi());

			if (nvs_DAVIS_DEBUG) uart_app_sendData(pcTaskGetName(0), buf, len);

			if (DavisRFM69_parseData(buf, &davis_data)){
				//ESP_LOGI(pcTaskGetName(0), "ID: %d", davis_data.id );
				//ESP_LOGI(pcTaskGetName(0), "Battery: %d", davis_data.battery );
				//ESP_LOGI(pcTaskGetName(0), "Wind Speed: %d   Wind Direction: %d", davis_data.wind_speed, davis_data.wind_direction );
				//ESP_LOGI(pcTaskGetName(0), "Type: %d", davis_data.type );
				//mean_sender_save_data(davis_data);
				if (davis_data.id != DavisRFM69_listening_station()) ESP_LOGE(pcTaskGetName(0), "Invalid Station! (%d)", davis_data.id);
				else {
					DavisRFM69_setChannel(++currChannel);
					//ESP_LOGI(pcTaskGetName(0), "ID: %d", davis_data.id );
					//ESP_LOGI(pcTaskGetName(0), "Battery: %d", davis_data.battery );
					mean_sender_save_data(davis_data);
				}

			}else ESP_LOGE(pcTaskGetName(0), "Invalid Data!");



		} else {
			ESP_LOGE(pcTaskGetName(0), "Receive failed");
		} // end recv
	} // end available
	else ESP_LOGE(pcTaskGetName(0), "Nothing to read!");

	if (pvParameter == NULL) vTaskDelete(NULL);
	// Removed for Light Sleep task!
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    xTaskCreate(&rx_task, "rx_task", 1024*3, NULL, 1, NULL);
}

void set_send_interval(uint16_t new_send_interval){
	nvs_SEND_INTERVAL = new_send_interval;

	nvs_app_set_uint16_value(NVS_SEND_INTERVAL_KEY, nvs_SEND_INTERVAL);
}

void set_appEui(const char* new_appEui){
	strncpy(nvs_APPEUI, new_appEui, 16);
	strcat(nvs_APPEUI, "\0");

	ESP_LOGI(TAG,"New AppEui: %s", nvs_APPEUI);

	nvs_app_set_string_value(NVS_APPEUI_KEY, nvs_APPEUI);
}

void set_devEui(const char* new_devEui){
	strncpy(nvs_DEVEUI, new_devEui, 16);
	strcat(nvs_DEVEUI, "\0");

	ESP_LOGI(TAG,"New DevEui: %s", nvs_DEVEUI);

	nvs_app_set_string_value(NVS_DEVEUI_KEY, nvs_DEVEUI);
}

void set_appKey(const char* new_appKey){
	strncpy(nvs_APPKEY, new_appKey, 32);
	strcat(nvs_APPKEY, "\0");

	ESP_LOGI(TAG,"New AppKey: %s", nvs_APPKEY);

	nvs_app_set_string_value(NVS_APPKEY_KEY, nvs_APPKEY);
}

void set_listen_stations(const bool new_stations[8]){
	memcpy(nvs_LISTEN_STATIONS, new_stations, sizeof(bool) * 8);

	nvs_app_set_blob_value(NVS_LISTEN_STATIONS_KEY, nvs_LISTEN_STATIONS, 8);
}

void set_davis_debug(uint8_t new_davis_debug){
	nvs_DAVIS_DEBUG = new_davis_debug;

	nvs_app_set_uint8_value(NVS_DAVIS_DEBUG_KEY, nvs_DAVIS_DEBUG);
}

void init_properties(){
	esp_err_t res;
	size_t size;


	res = nvs_app_get_uint16_value(NVS_SEND_INTERVAL_KEY, &nvs_SEND_INTERVAL);

	if (res == ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGW(TAG, "Restored nvs_SEND_INTERVAL to default");
		set_send_interval(SEND_INTERVAL_DEFAULT);
	}

	res = nvs_app_get_string_value(NVS_APPEUI_KEY, NULL, &size);
	if (res == ESP_ERR_NVS_NOT_FOUND || size == 0 || size > 17){
		ESP_LOGW(TAG, "Restored nvs_APPEUI to default");
		set_appEui(APPEUI_DEFAULT);
	}else {
		nvs_app_get_string_value(NVS_APPEUI_KEY, nvs_APPEUI, &size);
	}

	res = nvs_app_get_string_value(NVS_DEVEUI_KEY, NULL, &size);
	if (res == ESP_ERR_NVS_NOT_FOUND || size == 0 || size > 17){
		ESP_LOGW(TAG, "Restored nvs_DEVEUI to default");
		set_devEui(DEVEUI_DEFAULT);
	}else nvs_app_get_string_value(NVS_DEVEUI_KEY, nvs_DEVEUI, &size);

	res = nvs_app_get_string_value(NVS_APPKEY_KEY, NULL, &size);
	if (res == ESP_ERR_NVS_NOT_FOUND || size == 0 || size > 33){
		ESP_LOGW(TAG, "Restored nvs_APPKEY to default");
		set_appKey(APPKEY_DEFAULT);
	}else  nvs_app_get_string_value(NVS_APPKEY_KEY, nvs_APPKEY, &size);

	res = nvs_app_get_blob_value(NVS_LISTEN_STATIONS_KEY, NULL, &size);
	if (res == ESP_ERR_NVS_NOT_FOUND || size == 0 || size > 8){
		ESP_LOGW(TAG, "Restored nvs_LISTEN_STATIONS to default");
		set_listen_stations(NVS_LISTEN_STATIONS_DEFAULT);
	}else  nvs_app_get_blob_value(NVS_LISTEN_STATIONS_KEY, nvs_LISTEN_STATIONS, &size);

	res = nvs_app_get_uint8_value(NVS_DAVIS_DEBUG_KEY, &nvs_DAVIS_DEBUG);
	if (res == ESP_ERR_NVS_NOT_FOUND){
		ESP_LOGW(TAG, "Restored nvs_DAVIS_DEBUG to default");
		set_davis_debug(DAVIS_DEBUG_DEFAULT);
	}
}

void lorawan_set_conf(const char *appEui, const char *devEui, const char *appKey){
	if(strcmp(nvs_APPEUI,appEui) != 0){
		set_appEui(appEui);
	}

	if(strcmp(nvs_DEVEUI,devEui) != 0){
		set_devEui(devEui);
	}

	if(strcmp(nvs_APPKEY,appKey) != 0){
		set_appKey(appKey);
	}
}

esp_err_t init_ttn(){
    // Initialize TTN
    ttn_init();

    // Configure the SX127x pins
    ttn_configure_pins(TTN_SPI_HOST, TTN_PIN_NSS, TTN_PIN_RXTX, TTN_PIN_RST, TTN_PIN_DIO0, TTN_PIN_DIO1);

    // The below line can be commented after the first run as the data is saved in NVS
    ttn_provision(nvs_DEVEUI, nvs_APPEUI, nvs_APPKEY);

    //ttn_on_message(messageReceived);

   if(ttn_join()){
	   //ttn_prepare_for_deep_sleep();
	   return ESP_OK;
   }
   else return ESP_FAIL;
}

void init_TTN_modem(){
    // Initialize SPI bus
    spi_bus_config_t spi_bus_config = {
        .miso_io_num = TTN_PIN_SPI_MISO,
        .mosi_io_num = TTN_PIN_SPI_MOSI,
        .sclk_io_num = TTN_PIN_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    ESP_ERROR_CHECK(spi_bus_initialize(TTN_SPI_HOST, &spi_bus_config, TTN_SPI_DMA_CHAN));

	gpio_reset_pin(CONFIG_NSS_GPIO);
	gpio_set_direction(CONFIG_NSS_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_NSS_GPIO, 1);


	ESP_ERROR_CHECK(init_ttn());
	ttn_wait_for_idle();
	//ttn_prepare_for_deep_sleep();
	//ttn_shutdown();

	lorawan_status = 1;
}

void init_RFM69_modem(){
	DavisRFM69_reset();
	while(!DavisRFM69_init()) {
		ESP_LOGE(TAG, "RFM69 radio init failed");
		vTaskDelay(pdMS_TO_TICKS(10000));
	}
	DavisRFM69_setStations(nvs_LISTEN_STATIONS);
	DavisRFM69_setChannel(0);
	DavisRFM69_hop_station();
	ESP_LOGI(TAG,"Listening to STATION_%d", DavisRFM69_listening_station());

	ESP_LOGI(TAG, "RFM69 radio init OK!");

	//DavisRFM69_setSleep();
    gpio_config_t input_pin_config = {
        .pin_bit_mask = BIT64(14),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = false,
        .pull_down_en = true,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&input_pin_config);
	gpio_isr_handler_add(14, gpio_isr_handler, (void*) 14);

	davis_status = 1;
}

static void stop_server(void* arg){
	ESP_LOGW(TAG, "stop_server: stopping connections...");
	wifi_manager_disconnect();
	http_server_stop();
	ESP_LOGW(TAG, "stop_server: WiFi & HTTP server disconnected!");

	ENTER_SLEEP_MODE = true;
}

void start_stop_server_timer(){
	esp_timer_handle_t stop_timer;

    const esp_timer_create_args_t stop_timer_args = {
            .callback = &stop_server,
            .name = "stop_server"
    };

    esp_timer_init();

    ESP_ERROR_CHECK(esp_timer_create(&stop_timer_args, &stop_timer));

    ESP_ERROR_CHECK(esp_timer_start_once(stop_timer, 10 * 60 * 1000000));
}

void app_main()
{
	nvs_app_init();
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
	ESP_ERROR_CHECK(esp_event_loop_create_default());

	init_properties();

	if (nvs_DAVIS_DEBUG) {
		ESP_LOGW("app_main", "DAVIS_DEBUG DETECTED");
		uart_app_init();
	}

	wifi_manager_init_softap();

	http_server_start();

	init_TTN_modem();

	init_RFM69_modem();

	start_mean_sender(nvs_SEND_INTERVAL);

	start_stop_server_timer();

	while (!ENTER_SLEEP_MODE){
		vTaskDelay(100);
	}

	if (mean_sender_status()){
		ESP_LOGW(TAG, "stop_server: Waiting to data to be sent!");
		while(mean_sender_status()){
			vTaskDelay(1000);
		}
	}
	stop_mean_sender();
    gpio_isr_handler_remove(14);
    gpio_set_intr_type(14, GPIO_INTR_DISABLE);

	ESP_ERROR_CHECK(gpio_set_pull_mode(14, GPIO_PULLDOWN_ONLY));
	ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(GPIO_NUM_14, 1));

	light_sleep_params_t params = {
	    .wakeup_timer = nvs_SEND_INTERVAL * 1000000,
	    .timer_handler = (void *) send_data_task,
	    .gpio_handler = (void *) rx_task
	};

	xTaskCreate(light_sleep_task, "lightSleep_task", 4096, &params, 6, NULL);

	vTaskDelete(NULL);
}


void lorawan_get_conf(char *appEui, char *devEui, char *appKey, uint8_t *status){
	strcpy(appEui, nvs_APPEUI);
	strcpy(devEui, nvs_DEVEUI);
	strcpy(appKey, nvs_APPKEY);

	*status = lorawan_status;
}

void davis_set_conf(uint16_t send_interval, bool stations[8], bool debug_trace){
	if (send_interval != nvs_SEND_INTERVAL) set_send_interval(send_interval);

	set_listen_stations(stations);

	if (debug_trace){
		set_davis_debug(1);
	}else set_davis_debug(0);
}

void davis_get_conf(uint16_t *send_interval, bool stations[8], uint8_t *status, bool *debug_trace){
	*send_interval = nvs_SEND_INTERVAL;

	memcpy(stations, nvs_LISTEN_STATIONS, 8);

	*status = davis_status;

	*debug_trace = nvs_DAVIS_DEBUG;
}

#endif
