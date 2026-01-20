idf_component_register(SRCS "tfg_mk3.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_timer wifi_component mqtt_component)