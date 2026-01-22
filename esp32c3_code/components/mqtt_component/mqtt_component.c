/* MQTT (over TCP) Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT_APP";
static esp_mqtt_client_handle_t global_client; // Referencia global para publicar
int angulo_recibido_mqtt = 0;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado al Broker");
            // Aquí puedes suscribirte a los comandos de la Raspberry Pi
            esp_mqtt_client_subscribe(global_client, "SPI_project/comandos", 0);
            break;
        case MQTT_EVENT_DATA:
            // 1. Creamos un buffer temporal para el comando y lo terminamos en NULL
            char buf[16]; 
            int len = (event->data_len < sizeof(buf) - 1) ? event->data_len : sizeof(buf) - 1;
            memcpy(buf, event->data, len);
            buf[len] = '\0';

            // 2. Convertimos el texto a número entero (función atoi)
            int valor = atoi(buf);

            // 3. Validamos el rango (-90 a 90)
            if (valor >= -90 && valor <= 90) {
                angulo_recibido_mqtt = valor;
                ESP_LOGI(TAG, "Nuevo ángulo aceptado: %d", angulo_recibido_mqtt);
            } else {
                ESP_LOGW(TAG, "Comando ignorado: valor %d fuera de rango (-90 a 90)", valor);
            }
            break;
        default:
            break;
    }
}

// Función que llamarás desde el main
void iniciar_mqtt(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_BROKER_URL,
    };
    global_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(global_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(global_client);
}

// NUEVA FUNCIÓN: Para publicar desde cualquier sitio
void mqtt_publicar(const char *topic, const char *data) {
    if (global_client != NULL) {
        esp_mqtt_client_publish(global_client, topic, data, 0, 1, 0);
    }
}
