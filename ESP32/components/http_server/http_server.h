#ifndef COMPONENTS_HTTP_SERVER_HTTP_SERVER_H_
#define COMPONENTS_HTTP_SERVER_HTTP_SERVER_H_

#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "sys/param.h"

#define HTTP_SERVER_TASK_STACK_SIZE		8192
#define HTTP_SERVER_TASK_PRIORITY		4
#define HTTP_SERVER_TASK_CORE_ID		0

#define HTTP_SERVER_MONITOR_STACK_SIZE	4096
#define HTTP_SERVER_MONITOR_PRIORITY	3
#define HTTP_SERVER_MONITOR_CORE_ID		0

#define OTA_UPDATE_PENDING		0
#define OTA_UPDATE_SUCCESSFUL	1
#define	OTA_UPDATE_FAIL			-1

/**
 * Messages for the HTTP Monitor
 */
typedef enum{
	HTTP_MSG_WIFI_CONNECT_INIT = 0,
	HTTP_MSG_WIFI_CONNECT_SUCCESS,
	HTTP_MSG_WIFI_CONNECT_FAIL,
	HTTP_MSG_OTA_UPDATE_SUCCESS,
	HTTP_MSG_OTA_UPDATE_FAIL
}http_server_msg_e;


typedef struct{
	http_server_msg_e msgID;
}http_server_queue_msg_t;


/**
 * Sends a message to the queue
 * @param msgID message ID from the http_server_msg_e enum
 * @return pdTRUE if an item was successfully sent to the queue, otherwise pdFALSE
 */
BaseType_t http_server_monitor_send_message(http_server_msg_e msgID);

/**
 * Starts the HTTP server
 */
void http_server_start(void);

/**
 * Stops the HTTP sevrer
 */
void http_server_stop(void);

/**
 * Timer callback function which calls esp_restart upon successful firmware update
 */
void http_server_fw_update_reset_callback(void *arg);

#endif /* COMPONENTS_HTTP_SERVER_HTTP_SERVER_H_ */
