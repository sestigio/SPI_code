idf_component_register(SRCS "wifi_component.c"
                       INCLUDE_DIRS "include" # O "." si el .h está junto al .c
                       REQUIRES esp_wifi esp_event nvs_flash) # Dependencias que use este código