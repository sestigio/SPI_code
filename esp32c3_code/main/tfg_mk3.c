#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h" //uso de tasks, delays y temporizadores
#include "freertos/task.h" 
#include "freertos/timers.h" 
#include "esp_log.h" // transmisión de mensajes al serial mejorada y detección de errores de ESP-IDF
#include "driver/ledc.h" // uso de pwm
#include "driver/gpio.h" // control de pines
#include "driver/adc.h" // uso de adc de la placa
#include "driver/i2c.h" // para conexión con mpu6050
#include "esp_timer.h" // para timer estilo millis()

#include <ada_hal.h>
#include <wifi_component.h>
#include <mqtt_component.h>

#define DUTY_INITIAL 307 // Para configurar PWM del Servo

const __uint8_t MPU_ADDR = 0x68;
const __uint8_t INA_ADDR = 0x40;
const __uint8_t TCA_ADDR = 0x70;
const int I2C_NUM = 0;
const int PIN_LED_VERDE = 10;
const int PIN_LED_AMARILLO = 1;
const int PIN_LED_ROJO = 0;
const int PIN_I2C_SCL = 5;
const int PIN_I2C_SDA = 4;
const int PIN_JOYSTICK_ADC = 3;
const int PIN_SERVO1_PWM = 2;
const int PIN_SERVO2_PWM = 8;
const int ADC_BUFFER_SIZE = 10;
const int PIN_BUTTON1 = 6;
const int PIN_BUTTON2 = 7;
const int MPU1_CHANNEL = 6;
const int MPU2_CHANNEL = 4;
const int MPU3_CHANNEL = 0;

int frequency = 50; // Para configurar PWM del Servo

//====== VARIABLES GLOBALES AÑADIDAS PARA TELEMETRÍA========//
float x1_accel = 0, y1_accel = 0, z1_accel = 0;
float x2_accel = 0, y2_accel = 0, z2_accel = 0;
float x3_accel = 0, y3_accel = 0, z3_accel = 0;

float servo1_shunt_voltage = 0, servo1_current = 0, servo1_bus_voltage = 0; 
float servo2_shunt_voltage = 0, servo2_current = 0, servo2_bus_voltage = 0;
//=========================================================//

uint64_t t_Intr1 = 0;
uint64_t t_Intr2 = 0;
int interval_t_Intr1 = 250000; // Para evitar los rebotes delbotón en la interrupción

bool state_change = false;
bool button1_pressed = false;
bool button2_pressed = false;

static const char* TAG = "TFG_Main";

void __gnat_last_chance_handler(char *source_location,
    int line)
{
    while (1){
        ESP_LOGE(TAG, "Gnat_last_chance_handler called. Source: %s , Line: %d\n", source_location, line);
        continue;
    }
}

void mqtt_enviar_telemetria(void) {

    char json_buffer[512]; // Tamaño suficiente para el string

    // Creamos el JSON manualmente
    snprintf(json_buffer, sizeof(json_buffer),
        "{\"curr_st\":%d,\"prior_st\":%d,\"servo_st\":%d,\"gyro_st\":%d,\"mean\":%d,\"bearing\":%d,\"x_accel_glob\":%.2f,\"y_accel_glob\":%.2f,\"z_accel_glob\":%.2f,\"x1_accel\":%.2f,\"y1_accel\":%.2f,\"z1_accel\":%.2f,\"x2_accel\":%.2f,\"y2_accel\":%.2f,\"z2_accel\":%.2f,\"x3_accel\":%.2f,\"y3_accel\":%.2f,\"z3_accel\":%.2f,\"s1_v_sh\":%.2f,\"s1_curr\":%.2f,\"s1_v_bus\":%.2f,\"s2_v_sh\":%.2f,\"s2_curr\":%.2f,\"s2_v_bus\":%.2f}",
        current_state, prior_state, servo_state, gyro_state, mobile_mean, servo_bearing, x_accel, y_accel, z_accel, x1_accel, y1_accel, z1_accel, x2_accel, y2_accel, z2_accel, x3_accel, y3_accel, z3_accel,servo1_shunt_voltage, servo1_current, servo1_bus_voltage, servo2_shunt_voltage, servo2_current, servo2_bus_voltage);

    // Reutilizamos tu función de publicar
    mqtt_publicar("SPI_project/telemetria", json_buffer);
}

void ada_esp_log(int debug_code)
{
    switch (debug_code)
    {
    case 100:
        ESP_LOGE(TAG, "Unknown state\n");
        break;
    case 101:
        printf("\nCurrent state: %d\n", current_state);
        break;
    case 102:
        printf("\nCurrent averaged reading: %d\n", mobile_mean);
        break;
    case 103:
        printf("\nCurrent servo bearing: %d\n", servo_bearing);
        break;
    case 104:
        ESP_LOGI(TAG, "Current servo state: %d\n", servo_state);
        break;
    case 105:
        ESP_LOGI(TAG, "MPU has been read\n");
        break;
    case 106:
        ESP_LOGI(TAG, "MPU reading has been processed\n");
        break;
    case 107:
        ESP_LOGI(TAG, "MPU reading has been computed\n");
        break;
    case 108:
        ESP_LOGI(TAG, "MPU initialized successfully\n");
        break;
    case 109:
        ESP_LOGI(TAG, "INA initialized successfully\n");
        break;
    case 110:
        ESP_LOGE(TAG, "All mpus are faulty\n");
        break;
    case 111:
        ESP_LOGI(TAG, "All MPUs are nominal\n");
        break;
    case 112:
        ESP_LOGW(TAG, "MPU %d is faulty\n", gyro_state);
        break;
    case 113:
        ESP_LOGE(TAG, "Unknown mpu number\n");
        break;
    case 114:
        ESP_LOGI(TAG, "Assessing results\n");
        break;
    case 115:
        ESP_LOGI(TAG, "Results assessed\n");
        break;
    case 116:
        ESP_LOGI(TAG, "Operating nominal servo\n");
        break;
    case 117:
        ESP_LOGW(TAG, "Operating redundant servo\n");
        break;
    case 118:
        printf("\n-------------------------------------\n");
        ESP_LOGI(TAG, "\nStarting info output\n");
        break;
    case 119:
        ESP_LOGI(TAG, "\nStarting INA output\n");
        break;
    default:
        break;
    }
}

void ada_esp_log_gyro(float x, float y, float z, float t, float gx, float gy, float gz)
{
    printf("\nx=%.2f , y=%.2f , z=%.2f\nT=%.2f\nGx=%.2f , Gy=%.2f , Gz=%.2f\n", x, y, z, t, gx, gy, gz);
}

void ada_esp_log_local_accel(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3)
{
    x1_accel = x1;
    y1_accel = y1;
    z1_accel = z1;
    x2_accel = x2;
    y2_accel = y2;
    z2_accel = z2;
    x3_accel = x3;
    y3_accel = y3;
    z3_accel = z3;
}

void ada_esp_log_accel(float x, float y, float z)
{
    printf("\nNet accel: x=%.2f, y=%.2f, z=%.2f\n", x, y, z);
}

void ada_esp_log_ina(float c3_shunt_voltage, float c3_current, float c3_bus_voltage, float c2_shunt_voltage, float c2_current, float c2_bus_voltage)
{
    printf("\nSV1=%.3f, SC1= %.3f, BV1= %.3f\n", c3_shunt_voltage, c3_current, c3_bus_voltage);
    printf("\nSV2=%.3f, SC2= %.3f, BV2= %.3f\n", c2_shunt_voltage, c2_current, c2_bus_voltage);

    servo1_shunt_voltage = c3_shunt_voltage;
    servo1_bus_voltage = c3_bus_voltage;
    servo1_current = c3_current;   
    servo2_shunt_voltage = c2_shunt_voltage;
    servo2_bus_voltage = c2_bus_voltage;
    servo2_current = c2_current;
}

//-------------- FUNCIONES DE LED ---------------//

esp_err_t init_led(void)
{
    gpio_config_t pGPIOConfig =
    {
        .pin_bit_mask = ((1ULL<<PIN_LED_ROJO) | (1ULL<<PIN_LED_AMARILLO) | (1ULL<<PIN_LED_VERDE)),                       /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_OUTPUT,               /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_ENABLE,           /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE,         /*!< GPIO interrupt type                                  */
    };

    gpio_config(&pGPIOConfig);

    return ESP_OK;
}

//-------------- FUNCIONES DE BOTÓN ---------------//

void button1_isr_handle(void *args) // Función de interrupción de botón 1
{
    if ( esp_timer_get_time() >= t_Intr1 + interval_t_Intr1 ) 
    {
        t_Intr1 = esp_timer_get_time();
        state_change = true;
        button1_pressed = true;
    }
}

void button2_isr_handle(void *args) // Función de interrupción de botón 2
{
    if ( esp_timer_get_time() >= t_Intr2 + interval_t_Intr1 ) 
    {
        t_Intr2 = esp_timer_get_time();
        button2_pressed = true;
    }
}

esp_err_t init_buttons(void)
{
    gpio_config_t pGPIOConfig =
    {
        .pin_bit_mask = ((1ULL<<PIN_BUTTON1) | (1ULL<<PIN_BUTTON2)),                       /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_INPUT,               /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_ENABLE,           /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_NEGEDGE,         /*!< GPIO interrupt type                                  */
    };

    gpio_config(&pGPIOConfig);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_BUTTON1, button1_isr_handle, NULL);
    gpio_isr_handler_add(PIN_BUTTON2, button2_isr_handle, NULL);
    ESP_LOGI(TAG, "Interrupt setup completed\n");
    return ESP_OK;
}

//-------------- FUNCIONES DE JOYSTICK ---------------//

esp_err_t init_adc(void)
{
    adc1_config_channel_atten(PIN_JOYSTICK_ADC, ADC_ATTEN_DB_12);
    adc1_config_width(ADC_WIDTH_MAX);
    ESP_LOGI(TAG, "ADC setup completed\n");
    return ESP_OK;
}

//-------------- FUNCIONES DE SERVO ---------------//

esp_err_t init_pwm(void)
{
    ledc_channel_config_t channelConfig1_servo = {0};
    channelConfig1_servo.gpio_num = PIN_SERVO1_PWM;
    channelConfig1_servo.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig1_servo.channel = LEDC_CHANNEL_0;
    channelConfig1_servo.intr_type = LEDC_INTR_DISABLE;
    channelConfig1_servo.timer_sel = LEDC_TIMER_0;
    channelConfig1_servo.duty = DUTY_INITIAL;

    ledc_channel_config_t channelConfig2_servo = {0};
    channelConfig2_servo.gpio_num = PIN_SERVO2_PWM;
    channelConfig2_servo.speed_mode = LEDC_LOW_SPEED_MODE;
    channelConfig2_servo.channel = LEDC_CHANNEL_1;
    channelConfig2_servo.intr_type = LEDC_INTR_DISABLE;
    channelConfig2_servo.timer_sel = LEDC_TIMER_0;
    channelConfig2_servo.duty = DUTY_INITIAL;

    ledc_channel_config(&channelConfig1_servo);
    ledc_channel_config(&channelConfig2_servo);

    ledc_timer_config_t timerConfig_servo = {0};
    timerConfig_servo.speed_mode = LEDC_LOW_SPEED_MODE;
    timerConfig_servo.duty_resolution = LEDC_TIMER_12_BIT;
    timerConfig_servo.timer_num = LEDC_TIMER_0;
    timerConfig_servo.freq_hz = frequency;

    ledc_timer_config(&timerConfig_servo);

    return ESP_OK;
}

esp_err_t set_pwm_duty(int duty, int channel)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);

    return ESP_OK;
}

//-------------- FUNCIONES DE MPU ---------------//

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C_SDA,
        .scl_io_num = PIN_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 200000,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

/*
esp_err_t init_mpu(void)
{
    uint8_t data[1];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    //mpuReadfromReg(0x75, data, 1);
    ESP_LOGI(TAG, "%X", data[0]);
    //mpuWriteReg(0x6B, 0); // MPU configuration
    //mpuWriteReg(0x19, 7); // sample rate 1KHz
    //mpuWriteReg(0x1C, 0);  // ACC FS Range ±2g
    //mpuWriteReg(0x1B, 0);  // GYR FS Range ±250º/s
    ESP_LOGI(TAG, "MPU initialized successfully");

    return ESP_OK;
}

//-------------- FUNCIONES DE INA ---------------//

esp_err_t init_ina(void)
{
    uint8_t data[2];
    data[1] = 21;  // 0 001 010 1 = 21 : Canal3, 16 muestras, 2ms para bus
    data[0] = 111; // 01 101 111 = 111 : 2ms para shunt, modo continua shunt y bus
    //inaWriteReg(0x00, data, 2);
    ESP_LOGI(TAG, "INA initialized successfully");

    return ESP_OK;
}
    */

//-------------- FUNCIÓN APP_MAIN ---------------//

void app_main(void)
{
    //-------------- LED SETUP ---------------//
    init_led();

    //-------------- BUTTON SETUP ---------------//
    init_buttons();

    //-------------- MPU SETUP ---------------//
    i2c_master_init();
    //init_mpu();

    //-------------- SERVO SETUP ---------------//
    init_pwm();

    //-------------- ADC SETUP ---------------//
    init_adc();

    //-------------- INA SETUP ---------------//
    //init_ina();

    //============= WIFI SETUP ===============//
    iniciar_wifi();

    //============= MQTT SETUP ===============//
    iniciar_mqtt();

    // Call Ada elaboration code and auxiliary Ada initialization//
    
    Ada_Codeinit();

    //ada_auxiliary_init();
    
    ada_init_mpu();

    ada_init_ina();

    // ------------------------- //

    // VARIABLES DE INTERVALO TEMPORAL PARA ENVÍO MQTT
    int64_t last_send_time = 0;
    const int64_t send_interval = 1000000; // 1 segundo en microsegundos (1000ms)

    while (1)
    {        
        ada_code_exec(); 

        int64_t now = esp_timer_get_time();
        if ((now - last_send_time) >= send_interval) {
            mqtt_enviar_telemetria();
            last_send_time = now;
        }

    }

}

