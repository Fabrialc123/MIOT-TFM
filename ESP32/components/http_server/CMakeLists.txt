idf_component_register(SRCS "http_server.c" INCLUDE_DIRS "../../main" INCLUDE_DIRS "." REQUIRES wifi_app esp_http_server app_update esp_timer nvs_flash
						EMBED_FILES webpage/app.css 
							webpage/app.js  
							webpage/index.html 
							webpage/jquery-3.3.1.min.js)