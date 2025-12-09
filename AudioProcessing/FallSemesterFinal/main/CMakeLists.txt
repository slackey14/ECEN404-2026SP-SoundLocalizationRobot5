idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        nvs_flash 
                        esp_wifi 
                        esp_event 
                        esp_http_server 
                        json 
                        lwip 
                        driver 
                        esp_driver_i2s 
                        esp_timer)