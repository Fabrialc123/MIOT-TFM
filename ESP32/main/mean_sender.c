#ifndef RH_MEAN_SENDER_C
#define RH_MEAN_SENDER_C

#include "esp_timer.h"
#include "Davis-RFM69.h"
#include "ttn.h"

static esp_timer_handle_t send_timer;
static bool SENDING_DATA = false;
static bool SENDING_DELAYED = false;

static struct mean_davis_data_t {
    uint8_t id;
    uint8_t battery;

    uint32_t wind_speed;
    uint32_t wind_dir;
    uint8_t wind_count;

    float uv;
    uint8_t uv_count;
    uint8_t uv_act;

    float solar_rad;
    uint8_t solar_rad_count;
    uint8_t solar_rad_act;

    float temp;
    uint8_t temp_count;
    uint8_t temp_act;

    float hum;
    uint8_t hum_count;
    uint8_t hum_act;

    float rain;
    uint8_t rain_count;
    uint8_t rain_act;
} mean_davis_data;

void mean_sender_reset_data(){
	mean_davis_data.id = DavisRFM69_listening_station();
	mean_davis_data.battery = 0;

	mean_davis_data.wind_count = 0;
	mean_davis_data.wind_dir = 0;
	mean_davis_data.wind_speed = 0;

	mean_davis_data.hum = 0;
	mean_davis_data.hum_act = 0;
	mean_davis_data.hum_count = 0;

	mean_davis_data.rain = 0;
	mean_davis_data.rain_act = 0;
	mean_davis_data.rain_count = 0;

	mean_davis_data.solar_rad = 0;
	mean_davis_data.solar_rad_act = 0;
	mean_davis_data.solar_rad_count = 0;

	mean_davis_data.temp = 0;
	mean_davis_data.temp_act = 0;
	mean_davis_data.temp_count = 0;

	mean_davis_data.uv = 0;
	mean_davis_data.uv_act = 0;
	mean_davis_data.uv_count = 0;
}

void mean_sender_print_data(){
	ESP_LOGI(pcTaskGetName(0), "Station: %d (%d)", mean_davis_data.id, mean_davis_data.battery);
	ESP_LOGI(pcTaskGetName(0), "Wind Speed: %f \t Wind Dir: %f \t Count: %d"
				, ((mean_davis_data.wind_speed / (double) mean_davis_data.wind_count))
				, ((mean_davis_data.wind_dir / (double) mean_davis_data.wind_count) / (360/255.0))
						, mean_davis_data.wind_count
				);

	ESP_LOGI(pcTaskGetName(0), "Hum: %.2f \t(%d/%d)"
				, ((mean_davis_data.hum / (double) (mean_davis_data.hum_act)))
				, mean_davis_data.hum_act
				, mean_davis_data.hum_count);

	ESP_LOGI(pcTaskGetName(0),"Rain: %.2f \t(%d/%d)"
				, ((mean_davis_data.rain / (double) (mean_davis_data.rain_act)))
				, mean_davis_data.rain_act
				, mean_davis_data.rain_count);

	ESP_LOGI(pcTaskGetName(0),"Solar Rad: %.2f \t(%d/%d)"
				, ((mean_davis_data.solar_rad / (double) (mean_davis_data.solar_rad_act)))
				, mean_davis_data.solar_rad_act
				, mean_davis_data.solar_rad_count);

	ESP_LOGI(pcTaskGetName(0),"Temp: %.2f \t (%.2f)(%d/%d)"
				, ((mean_davis_data.temp / (double) (mean_davis_data.temp_act)))
				, mean_davis_data.temp
				, mean_davis_data.temp_act
				, mean_davis_data.temp_count);

	ESP_LOGI(pcTaskGetName(0),"UV: %.2f \t(%d/%d)"
			,((mean_davis_data.uv / (double) (mean_davis_data.uv_act)))
			, mean_davis_data.uv_act
			, mean_davis_data.uv_count);

}

static void set_nullValue(uint8_t *buf, uint8_t *pos){
	uint8_t i;

	i = *pos;
	buf[i++] = 0xFF;
	buf[i++] = 0xFF;

	*pos = i;
}

uint8_t parse_data(uint8_t *buf){
	uint8_t len = 0;
	uint32_t aux2;

	buf[len++] = (mean_davis_data.battery << 7) | mean_davis_data.id;

	buf[len++] = mean_davis_data.wind_count;

	if (!mean_davis_data.wind_count) return len;

	aux2 = ((mean_davis_data.wind_speed / (double) mean_davis_data.wind_count)) * 100;
	buf[len++] = aux2 / 100;
	buf[len++] = aux2 % 100;

	aux2 = ((mean_davis_data.wind_dir / (double) mean_davis_data.wind_count) / (360/255.0)) * 100;
	buf[len++] = aux2 / 100;
	buf[len++] = aux2 % 100;

	if (mean_davis_data.hum_act){
		aux2 = (mean_davis_data.hum / (double) (mean_davis_data.hum_act)) * 100;
		buf[len++] = aux2 / 100;
		buf[len++] = aux2 % 100;
	}else set_nullValue(buf, &len);

	if (mean_davis_data.rain_act){
		aux2 = ((mean_davis_data.rain / (double) (mean_davis_data.rain_act))) * 100;
		buf[len++] = aux2 / 100;
		buf[len++] = aux2 % 100;
	}else set_nullValue(buf, &len);

	if (mean_davis_data.solar_rad_act){
		aux2 = (mean_davis_data.solar_rad / (double) (mean_davis_data.solar_rad_act)) * 100;
		buf[len++] = aux2 / 100;
		buf[len++] = aux2 % 100;
	}else set_nullValue(buf, &len);


	if (mean_davis_data.temp_act){
		aux2 = ((mean_davis_data.temp / (double) (mean_davis_data.temp_act)) + 120) * 100;
		buf[len++] = aux2 / 100;
		buf[len++] = aux2 % 100;
	}else set_nullValue(buf, &len);

	if (mean_davis_data.uv_act){
		aux2 = (mean_davis_data.uv / (double) (mean_davis_data.uv_act)) * 100;
		buf[len++] = aux2 / 100;
		buf[len++] = aux2 % 100;
	}else set_nullValue(buf, &len);

	//for (int i = 0; i < len; i++) ESP_LOGW(TAG, "BUF[%i]: %x", i, buf[i]);

	return len;

}

void send_data(uint8_t *buf, size_t len){
	int64_t start_time = esp_timer_get_time();
    ttn_response_code_t res = ttn_transmit_message(buf, len, 1, false);
    int64_t end_time = esp_timer_get_time();
    if (res == TTN_SUCCESSFUL_TRANSMISSION )ESP_LOGI(pcTaskGetName(0),"Message sent (%f s)", (end_time - start_time)/ 1000000.0);
    else ESP_LOGI(pcTaskGetName(0),"Transmission failed");
}

static void send_data_task(void* arg){
	uint8_t buf[16], len;
	len = parse_data(buf);
	mean_sender_print_data();
	mean_sender_reset_data();

	SENDING_DATA = true;

	ESP_LOGI(pcTaskGetName(0),"Sending data...");

	DavisRFM69_setSleep();

	send_data(buf, len);

	ESP_LOGI(pcTaskGetName(0),"Data sent, waiting ttn for idle");
	ttn_wait_for_idle();

	ESP_LOGI(pcTaskGetName(0),"Hop Station!");
	DavisRFM69_hop_station();
	ESP_LOGI(pcTaskGetName(0),"Listening to STATION_%d", DavisRFM69_listening_station());
	DavisRFM69_setChannel(DavisRFM69_channel());

	SENDING_DATA = false;

	if (arg == NULL) vTaskDelete(NULL);
	// Removed for Light Sleep task!
}

bool mean_sender_status(void){
	return SENDING_DATA;
}

static void send_timer_callback(void* arg){
	if (!SENDING_DATA) {
		xTaskCreate(&send_data_task, "send_data_task", 1024*3, NULL, 1, NULL);
		SENDING_DELAYED = false;
	}
	else if (SENDING_DELAYED) {
		ESP_LOGE("send_timer_callback", "A send was already delayed and can't be sent, restarting system");
		esp_restart();
	}else{
		ESP_LOGW("send_timer_callback", "Another data is still sending, delaying this send");
		SENDING_DELAYED = true;
	}
}



void start_mean_sender(uint16_t period){
    const esp_timer_create_args_t send_timer_args = {
            .callback = &send_timer_callback,
            .name = "mean_sender"
    };

    esp_timer_init();

    mean_sender_reset_data();

    SENDING_DATA = false;
    SENDING_DELAYED = false;

    ESP_ERROR_CHECK(esp_timer_create(&send_timer_args, &send_timer));

    ESP_ERROR_CHECK(esp_timer_start_periodic(send_timer, period * 1000000));
    //ESP_ERROR_CHECK(esp_timer_start_once(send_timer, period * 1000000));
}

void stop_mean_sender() {
    if (send_timer != NULL) {
        esp_timer_stop(send_timer);
        esp_timer_delete(send_timer);
        send_timer = NULL;
    }
}

void mean_sender_save_data(t_DavisData davis_data){
	mean_davis_data.wind_count++;

	//mean_davis_data.id = davis_data.id;
	mean_davis_data.battery += davis_data.battery;

	mean_davis_data.wind_dir += davis_data.wind_direction;
	mean_davis_data.wind_speed += davis_data.wind_speed;

	switch(davis_data.type){
		case 4:		// UV Index
			mean_davis_data.uv_count++;
			mean_davis_data.uv_act += davis_data.active? 1: 0;
			if (davis_data.active) mean_davis_data.uv += davis_data.data;
			break;

		case 6:		// Solar Radiation
			mean_davis_data.solar_rad_count++;
			mean_davis_data.solar_rad_act += davis_data.active? 1: 0;
			if (davis_data.active) mean_davis_data.solar_rad += davis_data.data;
			break;

		case 8:		// Temperature (ºF)
			mean_davis_data.temp_count++;
			mean_davis_data.temp_act += davis_data.active? 1: 0;
			if (davis_data.active) mean_davis_data.temp += davis_data.data;
			break;

		case 10:	// Humidity (%)
			mean_davis_data.hum_count++;
			mean_davis_data.hum_act += davis_data.active? 1: 0;
			if (davis_data.active) mean_davis_data.hum += davis_data.data;
			break;

		case 14:	// Rain
			mean_davis_data.rain_count++;
			mean_davis_data.rain_act += davis_data.active? 1: 0;
			if (davis_data.active) mean_davis_data.rain += davis_data.data;
			break;

		default:
			break;
		}
}

#endif


