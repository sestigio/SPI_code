-- in out solo usado para arrays
-- empleados subtipos para evitar nombres largos de variables incluidas en paquetes (y no teniendo que incluirlos enteros)
-- utilizados aspectos en lugar de pragmas
-- utilizados sufijos _t para tipos propios y _e para enumeraciones propias
-- utilizados parámetros out, con multiples ventajas de claridad, modularidad y facilidad de análisis formal
-- abreviaciones solo donde se entienda
with System;
with Interfaces; use Interfaces;
with Interfaces.C;
with Unchecked_Conversion;

package Ada_Code 
   with Elaborate_Body--, SPARK_Mode => On-- Para elaborar el body justo después de la especificación, inicializando así las variables que tiene después del begin y evitando que el código en C acceda antes a ellas
is

-- Tipos de variable -------------------------------------

use type Interfaces.Unsigned_8;
use type Interfaces.Integer_64;
use type Interfaces.Integer_16;

subtype UByte_t is Interfaces.Unsigned_8;
subtype UWord_t is Interfaces.Unsigned_16;
subtype UDWord_t is Interfaces.Unsigned_32;

subtype Byte_t is Interfaces.Integer_8;
subtype Word_t is Interfaces.Integer_16;
subtype DWord_t is Interfaces.Integer_32;

type UByte_buffer14_t is array (0 .. 13) of UByte_t;
type UByte_buffer2_t is array (0 .. 1) of UByte_t;

type UByte_buffer_t is array (UByte_t range <>) of UByte_t
   with Convention => C;

type buffer_mobile_mean_t is array (0 .. 9) of Integer;

type mpu_data_t is array (Integer range <>) of Float;
type mpu_difference_t is array (Integer range <>) of Integer;

type states_e is (NONE, STANDBY, MANUAL, AUTOMATIC);
for states_e'Size use Interfaces.C.int'Size;
for states_e use (NONE => 0, STANDBY => 1, MANUAL => 2, AUTOMATIC => 3);

type servo_states_e is (SERVO_FULL, SERVO_RESTRICTED, SERVO_REDUNDANT);
for servo_states_e'Size use Interfaces.C.int'Size;
for servo_states_e use (SERVO_FULL => 0, SERVO_RESTRICTED => 1, SERVO_REDUNDANT => 2);

type gyro_states_e is (GYRO_NORMAL, GYRO1_FAIL, GYRO2_FAIL, GYRO3_FAIL, ALL_GYRO_FAIL);
for gyro_states_e'Size use Interfaces.C.int'Size;
for gyro_states_e use (GYRO_NORMAL => 0, GYRO1_FAIL => 1, GYRO2_FAIL => 2, GYRO3_FAIL => 3, ALL_GYRO_FAIL => 4);

-- Variables disponibles solo en Ada -------------------------------------

duty_cycle : Integer range 0 .. 600;

buffer_mobile_mean : buffer_mobile_mean_t;

index10_read : Integer range 0 .. 9;

t_1 : Interfaces.Integer_64;
t_1_interval : Interfaces.Integer_64;

--t_2 : Interfaces.Integer_64;
--t_3 : Interfaces.Integer_64;

--green_led : Boolean;
--yellow_led : Boolean;
--red_led : Boolean;
x_accel, y_accel, z_accel : Float
   with Export, Convention => C;

-- Variables exportadas a C -------------------------------------

prior_state : states_e := NONE
   with Export, Convention => C;
current_state : states_e := STANDBY
   with Export, Convention => C;
servo_state : servo_states_e := SERVO_FULL
   with Export, Convention => C;
gyro_state : gyro_states_e := GYRO_NORMAL
   with Export, Convention => C;

mobile_mean : Integer range 0 .. 4600
   with Export, Convention => C;

servo_bearing : Integer range -100 .. 100
   with Export, Convention => C;

count_to_log : constant Integer := 30;            -- Contador de iteraciones, que en cierto umbral genera logs en el serial
counter : Integer range 0 .. 60;

-- Variables importadas de C ------------------------------------------------

state_change : Boolean
   with Import, Convention => C;

button1_pressed : Boolean
   with Import, Convention => C;

button2_pressed : Boolean
   with Import, Convention => C;

MPU_ADDR : constant UByte_t
   with Import, Convention => C, External_Name => "MPU_ADDR";

INA_ADDR : constant UByte_t
   with Import, Convention => C, External_Name => "INA_ADDR";

TCA_ADDR : constant UByte_t
   with Import, Convention => C, External_Name => "TCA_ADDR";

I2C_NUM : constant UByte_t
   with Import, Convention => C, External_Name => "I2C_NUM";

PIN_LED_VERDE : constant Integer
   with Import, Convention => C, External_Name => "PIN_LED_VERDE";

PIN_LED_AMARILLO : constant Integer
   with Import, Convention => C, External_Name => "PIN_LED_AMARILLO";

PIN_LED_ROJO : constant Integer
   with Import, Convention => C, External_Name => "PIN_LED_ROJO";

PIN_I2C_SCL : constant Integer
   with Import, Convention => C, External_Name => "PIN_I2C_SCL";

PIN_I2C_SDA : constant Integer
   with Import, Convention => C, External_Name => "PIN_I2C_SDA";

--PIN_SERVO_PWM : constant Integer
--   with Import, Convention => C, External_Name => "PIN_SERVO_PWM";

PIN_BUTTON1 : constant Integer
   with Import, Convention => C, External_Name => "PIN_BUTTON1";

PIN_BUTTON2 : constant Integer
   with Import, Convention => C, External_Name => "PIN_BUTTON2";

ADC_BUFFER_SIZE : constant Integer
   with Import, Convention => C, External_Name => "ADC_BUFFER_SIZE";

PIN_JOYSTICK_ADC : constant Integer
   with Import, Convention => C, External_Name => "PIN_JOYSTICK_ADC";

MPU1_CHANNEL : constant Integer
   with Import, Convention => C, External_Name => "MPU1_CHANNEL";

MPU2_CHANNEL : constant Integer
   with Import, Convention => C, External_Name => "MPU2_CHANNEL";

MPU3_CHANNEL : constant Integer
   with Import, Convention => C, External_Name => "MPU3_CHANNEL";

angulo_recibido_mqtt : Integer range -90 .. 90
   with Import, Convention => C, External_Name => "angulo_recibido_mqtt";

-- Funciones de los periféricos, algunas importadas de C -------------------------------------

-- FUNCIONES MISCELÁNEAS
procedure ada_esp_log (debug_code : Integer)
   with Import, Convention => C;

procedure ada_esp_log_gyro(x : Float; y : Float; z : Float; t : Float; gx : Float; gy : Float; gz : Float)
   with Import, Convention => C;

procedure ada_esp_log_accel(x : Float; y : Float; z : Float)
   with Import, Convention => C;

procedure ada_esp_log_local_accel(x1 : Float; y1 : Float; z1 : Float; x2 : Float; y2 : Float; z2 : Float; x3 : Float; y3 : Float; z3 : Float)
   with Import, Convention => C;

procedure ada_esp_log_ina(c3_shunt_voltage : Float; c3_current : Float; c3_bus_voltage : Float; c2_shunt_voltage : Float; c2_current : Float; c2_bus_voltage : Float)
   with Import, Convention => C;

function esp_timer_get_time return Interfaces.Integer_64
   with Import, Convention => C;

procedure vTaskDelay (ticks : Interfaces.Unsigned_32)
   with Import, Convention => C, External_Name => "vTaskDelay";

-- FUNCIONES DE LED

procedure gpio_set_level (gpio_num : Integer; level : UDWord_t)
   with Import, Convention => C;

function gpio_get_level (gpio_num : Integer) return Integer
   with Import, Convention => C;

-- FUNCIONES DE BOTÓN



-- FUNCIONES DE JOYSTICK

function adc1_get_raw(adc_channel_num : Integer) return Integer
   with Import, Convention => C;

function reading_mobile_mean return Integer;

-- FUNCIONES DE SERVO

procedure operate_servo_manual;

procedure operate_servo_auto;

procedure set_pwm_duty (duty : Integer; servo_redundancy : Integer)
   with Import, Convention => C;

-- FUNCIONES DE MPU

procedure i2c_master_write_to_device (i2c_num : UByte_t; slave_addr : UByte_t; write_array : in UByte_buffer_t; write_length : UByte_t; ticks_to_wait : UDWord_t)
   with Import, Convention => C;

procedure i2c_master_write_read_device (i2c_num : UByte_t; slave_addr : UByte_t; write_array : in UByte_buffer_t; write_length : UByte_t; read_array : in out UByte_buffer_t; read_length : UByte_t; ticks_to_wait : UDWord_t)
   with Import, Convention => C;

function To_Word is new Unchecked_Conversion
      (Source => UByte_buffer_t, 
       Target => Word_t);

function To_UWord is new Unchecked_Conversion
      (Source => Word_t, 
       Target => UWord_t);

function UWord_to_Word is new Unchecked_Conversion
      (Source => UWord_t, 
       Target => Word_t);

procedure mpuReadfromReg (mpu_register : UByte_t; read_data : in out UByte_buffer_t; read_data_length : UByte_t)
   with Export, Convention => C, External_Name => "mpuReadfromReg";

procedure mpuWriteReg (mpu_register : UByte_t; write_data : UByte_t)
   with Export, Convention => C, External_Name => "mpuWriteReg";

function mpu_get_channel (mpu_number : Integer) return UByte_t;

function process_buffer_to_word (input_buffer : UByte_buffer_t; LSB_in : UByte_t; MSB_in : UByte_t) return Word_t;

procedure mpu_get_data (mpu_i2c_channel : UByte_t; x_accel : out Float; y_accel : out Float; z_accel : out Float; temperature : out Float);

function Approx_Equal(A, B : Float) return Boolean;

function Majority_Of(mpu_data_arr : mpu_data_t;
                        Faulty_Index : out Integer;
                        faulty_mpu : in Integer) return Float;
   --with Side_Effects;

procedure mpu_operate (x_accel : out Float; y_accel : out Float; z_accel : out Float);

-- FUNCIONES DE INA

procedure inaReadfromReg (ina_register : UByte_t; read_data : in out UByte_buffer_t; read_data_length : UByte_t);
   --with Export, Convention => C, External_Name => "inaReadfromReg";

procedure inaWriteReg (ina_register : UByte_t; write_data : in UByte_buffer_t);
   --with Export, Convention => C, External_Name => "inaWriteReg";

-- FUNCIONES DE TCA

procedure tcaSelectChannel (tca_channel : UByte_t);

-- FUNCIONES DE ESTADO

procedure process_Standby_state;

procedure process_Manual_state;

procedure process_Automatic_state;


-- Función auxiliar de Ada para ser llamada en C antes del bucle -------------------------------------

procedure ada_auxiliary_init
   with Export, Convention => C, External_Name => "ada_auxiliary_init";

procedure ada_init_mpu
   with Export, Convention => C, External_Name => "ada_init_mpu";

procedure ada_init_ina
   with Export, Convention => C, External_Name => "ada_init_ina";

-- Función principal de Ada para ser llamada en C en el bucle -------------------------------------
procedure ada_code_exec
   with Export, Convention => C, External_Name => "ada_code_exec";

end Ada_Code;

