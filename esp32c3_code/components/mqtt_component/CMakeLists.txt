idf_component_register(SRCS "mqtt_component.c"
                       INCLUDE_DIRS "include" # O "." si el .h está junto al .c
                       REQUIRES mqtt esp_event) # Dependencias que use este código