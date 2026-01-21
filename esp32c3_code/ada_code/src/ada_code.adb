package body Ada_Code is

   -- FUNCIONES DE LEDS

   --procedure operate_leds return Integer is

   --t_2_interval : constant Interfaces.Integer_64 := 100000;
   --t_3_interval : constant Interfaces.Integer_64 := 500000;
   --t_4_interval : constant Interfaces.Integer_64 := 1000000;

   --end operate_leds;

   -- FUNCIONES DE JOYSTICK

   function reading_mobile_mean return Integer is

      averaged_reading, new_reading, index10_write : Integer;
   begin
      new_reading := adc1_get_raw (PIN_JOYSTICK_ADC);
      averaged_reading :=
        mobile_mean
        + ((new_reading - buffer_mobile_mean (index10_read))
           / ADC_BUFFER_SIZE);

      index10_write := index10_read;
      buffer_mobile_mean (index10_write) := new_reading;
      index10_read := (index10_read + 1) mod 10;

      return averaged_reading;

   end reading_mobile_mean;

   -- FUNCIONES DE SERVO

   procedure operate_servo_manual is

      max_duty_cycle : constant Integer := 490;
      min_duty_cycle : constant Integer := 110;

      max_mean_reading : constant Integer := 3500; -- 
      min_mean_reading : constant Integer :=
        700; -- correspondiente a 120 grados del potenciometro (2/5 del arco de 300 grados)desde el tope max
      -- 4092 - 1637 = 2455; (4092+2455)/2 = 3274; 4092-3274=818

      duty_cycle_half_difference   : constant Integer :=
        (max_duty_cycle - min_duty_cycle) / 2;
      duty_cycle_mean              : constant Integer :=
        (max_duty_cycle + min_duty_cycle) / 2;
      mean_reading_half_difference : constant Integer :=
        (max_mean_reading - min_mean_reading) / 2;
      mean_reading_mean            : constant Integer :=
        (max_mean_reading + min_mean_reading) / 2;

      duty_cycle_limit_correction : Integer;

      servo_redundancy : Integer := 0;

   begin

      if servo_state = SERVO_REDUNDANT then
         set_pwm_duty (0, servo_redundancy);
         servo_redundancy := 1;
      end if;

      mobile_mean := reading_mobile_mean;
      if mobile_mean > max_mean_reading then
         mobile_mean := max_mean_reading;
      elsif mobile_mean < min_mean_reading then
         mobile_mean := min_mean_reading;
      end if;

      if servo_state = SERVO_RESTRICTED then
         duty_cycle_limit_correction :=
         95; -- 190 corresponde a una limitación de 45 º de servo

      else
         duty_cycle_limit_correction := 0;
      end if;

      --ada_esp_log (102); -- Mostrar nueva media móvil
      duty_cycle :=
      ((mobile_mean - mean_reading_mean)
         * (duty_cycle_half_difference - duty_cycle_limit_correction))
      / mean_reading_half_difference
      + duty_cycle_mean; -- 120 grados (rango de mov del palillo) es al rango total del pot lo que 1636 a 4096

      set_pwm_duty (duty_cycle, servo_redundancy);
      --servo_bearing := (mobile_mean-1636-1227) * 90 / 1227; -- 2454/2 = 1227
      servo_bearing :=
        (duty_cycle - duty_cycle_mean) * 90 / duty_cycle_half_difference;

      if counter >= count_to_log then

         ada_esp_log (118); -- Inicio de muestreo en terminal

         case servo_state is  --Mostrar estado del servo
         when SERVO_REDUNDANT =>
            ada_esp_log (117);
         when others =>
            ada_esp_log (116);
         end case;

         ada_esp_log (103); -- Mostrar ángulo del servo

      end if;
      

   end operate_servo_manual;

   procedure operate_servo_auto is
   
      front_bearing : Float range -0.8 .. 0.8;
      accel_max : constant Float := 0.785;
      accel_min : constant Float := -0.785;

      max_duty_cycle : constant Integer := 490;
      min_duty_cycle : constant Integer := 110;

      duty_cycle_half_difference   : constant Integer :=
        (max_duty_cycle - min_duty_cycle) / 2;
      duty_cycle_mean              : constant Integer :=
        (max_duty_cycle + min_duty_cycle) / 2;

      servo_redundancy : Integer := 0;
   
   begin

      if servo_state = SERVO_REDUNDANT then
         set_pwm_duty (duty_cycle, servo_redundancy);
         servo_redundancy := 1;
      end if;

      --duty_cycle := leer_comando_rasp("duty");

      set_pwm_duty (duty_cycle, servo_redundancy);

      servo_bearing :=
      (duty_cycle - duty_cycle_mean) * 90 / duty_cycle_half_difference;

      if counter >= count_to_log then

         ada_esp_log (118); -- Inicio de muestreo en terminal

         case servo_state is  --Mostrar estado del servo
         when SERVO_REDUNDANT =>
            ada_esp_log (117);
         when others =>
            ada_esp_log (116);
         end case;

         ada_esp_log (103); -- Mostrar ángulo del servo

      end if;

   end operate_servo_auto;

   -- FUNCIONES DE MPU

   procedure mpuReadfromReg (mpu_register : UByte_t; read_data : in out UByte_buffer_t; read_data_length : UByte_t) is
   
      register_array : constant UByte_buffer_t := (1 => mpu_register);

   begin

      i2c_master_write_read_device (i2c_num => I2C_NUM, 
         slave_addr => MPU_ADDR, 
         write_array => register_array, 
         write_length => 1, 
         read_array => read_data, 
         read_length => read_data_length, 
         ticks_to_wait => 2000);

   end mpuReadfromReg;

   procedure mpuWriteReg (mpu_register : UByte_t; write_data : in UByte_t) is
   
      write_buffer : UByte_buffer_t (1 .. 2);  -- writeBuf[len+1];

   begin
      
      write_buffer(1) := mpu_register;
      write_buffer(2) := write_data;
      i2c_master_write_to_device (i2c_num => I2C_NUM, 
         slave_addr => MPU_ADDR, 
         write_array => write_buffer, 
         write_length => 2, 
         ticks_to_wait => 1000);

   end mpuWriteReg;

   function mpu_get_channel (mpu_number : Integer) return UByte_t is

   begin

      case mpu_number is
         when 1 =>
            return UByte_t(MPU1_CHANNEL);
         when 2 =>
            return UByte_t(MPU2_CHANNEL);
         when 3 =>
            return UByte_t(MPU3_CHANNEL);
         when others =>
            ada_esp_log (113);
            return 0;
      end case;

   end mpu_get_channel;

   function process_buffer_to_word
     (input_buffer : in UByte_buffer_t; LSB_in : UByte_t; MSB_in : UByte_t)
      return Word_t
   is

      temp_buffer : UByte_buffer_t (1 .. 2);

   begin

      temp_buffer (2) := input_buffer (LSB_in);
      temp_buffer (1) := input_buffer (MSB_in);

      -- Direct conversion from the array to Unsigned_16
      return To_Word (temp_buffer);

   end process_buffer_to_word;

   procedure mpu_get_data (mpu_i2c_channel : UByte_t; x_accel : out Float; y_accel : out Float; z_accel : out Float; temperature : out Float) is

      read_buffer : UByte_buffer_t (1 .. 14);

      raw_x_acceleration, raw_y_acceleration, raw_z_acceleration : Word_t;
      raw_temperature                                            : Word_t;
      raw_x_angular_acceleration, raw_y_angular_acceleration, raw_z_angular_acceleration : 
        Word_t;
      x_accel_local, y_accel_local, z_accel_local, temperature_local : Float;
      x_angular_acceleration, y_angular_acceleration, z_angular_acceleration :
        Float;

   begin

      tcaSelectChannel(mpu_i2c_channel);

      mpuReadfromReg (16#3B#, read_buffer, 14);
      --ada_esp_log (105);

      raw_x_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 1, MSB_in => 2);
      raw_y_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 3, MSB_in => 4);
      raw_z_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 5, MSB_in => 6);
      raw_temperature := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 7, MSB_in => 8);
      raw_x_angular_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 9, MSB_in => 10);
      raw_y_angular_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 11, MSB_in => 12);
      raw_z_angular_acceleration := process_buffer_to_word
          (input_buffer => read_buffer, LSB_in => 13, MSB_in => 14);
      --ada_esp_log (106);

      x_accel_local := Float (raw_x_acceleration) / 16384.0;
      y_accel_local := Float (raw_y_acceleration) / 16384.0;
      z_accel_local := Float (raw_z_acceleration) / 16384.0;
      temperature_local := Float (raw_temperature) / 340.0 + 36.53;
      x_angular_acceleration := Float (raw_x_angular_acceleration) / 131.0;
      y_angular_acceleration := Float (raw_y_angular_acceleration) / 131.0;
      z_angular_acceleration := Float (raw_z_angular_acceleration) / 131.0;

      x_accel := x_accel_local;
      y_accel := y_accel_local;
      z_accel := z_accel_local;
      temperature := temperature_local;

      if counter >= count_to_log then

         ada_esp_log_gyro
         (x  => x_accel_local,
            y  => y_accel_local,
            z  => z_accel_local,
            t  => temperature_local,
            gx => x_angular_acceleration,
            gy => y_angular_acceleration,
            gz => z_angular_acceleration);

      end if;

   end mpu_get_data;

   -- Approximate equality
   function Approx_Equal(A, B : Float) return Boolean is

   Tolerance : constant Float := 0.65; -- 40 grados de tolerancia

   begin
      return abs (A - B) <= Tolerance;
   end Approx_Equal;

   -- Voting with tolerance and fault detection
   function Majority_Of(mpu_data_arr : mpu_data_t;
                        Faulty_Index : out Integer;
                        faulty_mpu : in Integer) return Float is

      A : Float;-- := mpu_data_arr(1); 
      B : Float;-- := mpu_data_arr(2); 
      C : Float;-- := mpu_data_arr(3); 

   begin

      if faulty_mpu > 0 then

         if faulty_mpu = 1 then
            A := mpu_data_arr(2);
            B := mpu_data_arr(3); 
         elsif faulty_mpu = 2 then
            A := mpu_data_arr(1);
            B := mpu_data_arr(3); 
         else  -- faulty_mpu = 3
            A := mpu_data_arr(1);
            B := mpu_data_arr(2); 
         end if;

         if Approx_Equal(A, B) then
            Faulty_Index := 0;
            return (A + B) / 2.0;
         else
            Faulty_Index := -1;  -- No agreement, all differ
            return 0.0;
         end if;

      else -- no faulty mpus

         A := mpu_data_arr(1);
         B := mpu_data_arr(2); 
         C := mpu_data_arr(3);

         if Approx_Equal(A, B) then
            if not Approx_Equal(A, C) then
               Faulty_Index := 3;
               return (A + B) / 2.0;
            else
               Faulty_Index := 0;  -- No fault detected
               return (A + B + C) / 3.0;
            end if;
         elsif Approx_Equal(A, C) then
            Faulty_Index := 2;
            return (A + C) / 2.0;
         elsif Approx_Equal(B, C) then
            Faulty_Index := 1;
            return (B + C) / 2.0;
         else
            Faulty_Index := -1;  -- No agreement, all differ
            return 0.0;
         end if;

      end if;
   
   end Majority_Of;
      
   procedure mpu_operate (x_accel : out Float; y_accel : out Float; z_accel : out Float) is

      x_accel_arr, y_accel_arr, z_accel_arr, temperature_arr : mpu_data_t (1 .. 3);
      different_accel_arr : mpu_difference_t (1 .. 3);
      
      Found_Minus_One : Boolean := False;
      Unique_Non_Zero : Integer := 0;
      Unique_Set : Boolean := False;
      Multiple_Non_Zero_Values : Boolean := False;
      faulty_mpu : Integer range 0 .. 3;

      x_accel_result, y_accel_result, z_accel_result : Float;

   begin

      case gyro_state is
         when ALL_GYRO_FAIL =>
            if counter >= count_to_log then
               ada_esp_log (110);
            end if;
            x_accel := 0.0;
            y_accel := 0.0;
            z_accel := 0.0;
            return;
         when GYRO_NORMAL =>
            faulty_mpu := 0;
         when others =>
            faulty_mpu := gyro_states_e'Pos(gyro_state);
      end case;

      for i in 1 .. 3 loop
         vTaskDelay (1);
         mpu_get_data(mpu_get_channel(i), x_accel_arr(i), y_accel_arr(i), z_accel_arr(i), temperature_arr(i));
         --ada_esp_log (105);
      end loop;

      x_accel_result := Majority_Of(x_accel_arr, different_accel_arr(1), faulty_mpu);
      y_accel_result := Majority_Of(y_accel_arr, different_accel_arr(2), faulty_mpu);
      z_accel_result := Majority_Of(z_accel_arr, different_accel_arr(3), faulty_mpu);

      for V of different_accel_arr loop
         --ada_esp_log (114);
         if V /= 0 then

            if V = -1 then
               Found_Minus_One := True;
            elsif not Unique_Set then
               Unique_Non_Zero := V;
               Unique_Set := True;
            elsif V /= Unique_Non_Zero then
               Multiple_Non_Zero_Values := True;
            end if;
         end if;
         
      end loop;

      --ada_esp_log (115);
      -- Report conditions

      if Found_Minus_One or Multiple_Non_Zero_Values then
         --At least one value is -1 or two of them are different and non zero
         gyro_state := ALL_GYRO_FAIL;

      elsif Unique_Set then
         --There is a different non zero or -1 value in one or more sensors
         gyro_state := gyro_states_e'Val(Unique_Non_Zero);
         
      else 
         --All values are zero
         null;
      end if;

      -- Logs cada count_to_log ejecuciones

   if counter >= count_to_log then

      case gyro_state is
         when ALL_GYRO_FAIL =>
            ada_esp_log (100);
         when GYRO_NORMAL =>
            ada_esp_log (111);
         when others =>
            ada_esp_log (112);
      end case;

      ada_esp_log_accel
      (x  => x_accel_result,
      y  => y_accel_result,
      z  => z_accel_result);

   end if;

   x_accel := x_accel_result;
   y_accel := x_accel_result;
   z_accel := x_accel_result;

   end mpu_operate;

   -- FUNCIONES DE INA

   procedure inaReadfromReg (ina_register : UByte_t; read_data : in out UByte_buffer_t; read_data_length : UByte_t) is
   
      register_array : constant UByte_buffer_t := (1 => ina_register);

   begin

      i2c_master_write_read_device (i2c_num => I2C_NUM, 
         slave_addr => INA_ADDR, 
         write_array => register_array, 
         write_length => 1, 
         read_array => read_data, 
         read_length => read_data_length, 
         ticks_to_wait => 2000);

   end inaReadfromReg;

   procedure inaWriteReg (ina_register : UByte_t; write_data : in UByte_buffer_t) is
   
      write_buffer : UByte_buffer_t (0 .. write_data'Length);  -- writeBuf[len+1];

   begin
      
      write_buffer(0) := ina_register;
      write_buffer(1 .. write_data'Length) := write_data;
      i2c_master_write_to_device (i2c_num => I2C_NUM, 
         slave_addr => INA_ADDR, 
         write_array => write_buffer, 
         write_length => write_data'Length, 
         ticks_to_wait => 1000);

   end inaWriteReg;

   procedure operate_ina is

      read_buffer : UByte_buffer_t (1 .. 4);

      raw_c3_shunt_voltage : Word_t;
      raw_c3_bus_voltage : Word_t;

      trimmed_c3_shunt_voltage : UWord_t;
      trimmed_c3_bus_voltage : UWord_t;

      c3_shunt_voltage : Float;
      c3_current : Float;
      c3_bus_voltage : Float;

      raw_c2_shunt_voltage : Word_t;
      raw_c2_bus_voltage : Word_t;

      trimmed_c2_shunt_voltage : UWord_t;
      trimmed_c2_bus_voltage : UWord_t;

      c2_shunt_voltage : Float;
      c2_current : Float;
      c2_bus_voltage : Float;

   begin

      -- LECTURA DE CANAL 1: SERVOMOTOR NOMINAL

      inaReadfromReg (ina_register => 16#05#, read_data => read_buffer, read_data_length => 2);
   
      raw_c3_shunt_voltage := process_buffer_to_word
            (input_buffer => read_buffer, LSB_in => 1, MSB_in => 2);
      trimmed_c3_shunt_voltage := Shift_Right_Arithmetic(To_UWord(raw_c3_shunt_voltage), 3);
      c3_shunt_voltage := Float(UWord_to_Word(trimmed_c3_shunt_voltage)) * 0.04;    -- en mV
      c3_current := c3_shunt_voltage * 10.0;                                        -- en mA

      inaReadfromReg (ina_register => 16#06#, read_data => read_buffer, read_data_length => 2);
      
      raw_c3_bus_voltage := process_buffer_to_word
            (input_buffer => read_buffer, LSB_in => 1, MSB_in => 2);
      trimmed_c3_bus_voltage := Shift_Right_Arithmetic(To_UWord(raw_c3_bus_voltage), 3);
      c3_bus_voltage := Float(UWord_to_Word(trimmed_c3_bus_voltage)) * 0.008;       -- en V

      -- LECTURA DE CANAL 2: SERVO REDUNDANTE

      inaReadfromReg (ina_register => 16#03#, read_data => read_buffer, read_data_length => 2);
      
      raw_c2_shunt_voltage := process_buffer_to_word
            (input_buffer => read_buffer, LSB_in => 1, MSB_in => 2);
      trimmed_c2_shunt_voltage := Shift_Right_Arithmetic(To_UWord(raw_c2_shunt_voltage), 3);
      c2_shunt_voltage := Float(UWord_to_Word(trimmed_c2_shunt_voltage)) * 0.04; -- en mV
      c2_current := c2_shunt_voltage * 10.0;

      inaReadfromReg (ina_register => 16#04#, read_data => read_buffer, read_data_length => 2);
      
      raw_c2_bus_voltage := process_buffer_to_word
            (input_buffer => read_buffer, LSB_in => 1, MSB_in => 2);
      trimmed_c2_bus_voltage := Shift_Right_Arithmetic(To_UWord(raw_c2_bus_voltage), 3);
      c2_bus_voltage := Float(UWord_to_Word(trimmed_c2_bus_voltage)) * 0.008; -- en V


      if (c3_current >= 800.0) or (c3_bus_voltage < 4.0) or (c3_current <= 1.0) then
         servo_state := SERVO_REDUNDANT;
         current_state := MANUAL;
      end if;

      -- Logs cada count_to_log ejecuciones

      if counter >= count_to_log then
         counter := 0;

         ada_esp_log (119); --Starting ina output

         ada_esp_log_ina (c3_shunt_voltage => c3_shunt_voltage, 
                        c3_current => c3_current, 
                        c3_bus_voltage => c3_bus_voltage, 
                        c2_shunt_voltage => c2_shunt_voltage, 
                        c2_current => c2_current, 
                        c2_bus_voltage => c2_bus_voltage);

      end if;

      counter := counter + 1;

   end operate_ina;

   -- FUNCIONES DE TCA

   procedure tcaSelectChannel (tca_channel : UByte_t) is

      tca_channel_bitmask : UByte_t;
      write_buffer : UByte_buffer_t (1 .. 1);

   begin

      tca_channel_bitmask := Shift_Left (1, Natural(tca_channel));
      write_buffer (1) := tca_channel_bitmask;

      i2c_master_write_to_device (i2c_num => I2C_NUM, 
            slave_addr => TCA_ADDR, 
            write_array => write_buffer, 
            write_length => write_buffer'Length, 
            ticks_to_wait => 1000);

   end tcaSelectChannel;

   -- FUNCIONES DE ESTADO

   procedure process_Standby_state is

   begin

      -- Inicializar estado nuevo
      if prior_state /= current_state then
         prior_state := current_state;
         ada_esp_log (101);
         gpio_set_level (PIN_LED_AMARILLO, 1);
         servo_state := SERVO_FULL;
         gyro_state := GYRO_NORMAL;
         set_pwm_duty (0, 0);
         set_pwm_duty (0, 1);
      end if;

      -- Proceso interno del estado
      vTaskDelay (1);

      -- Revisar condiciones de transición
      if state_change and button1_pressed then
         current_state := MANUAL;
         state_change := false;
         button1_pressed := false;
      elsif button2_pressed then
         button2_pressed := False;
      end if;

      -- Resetear estado viejo
      if prior_state /= current_state then
         gpio_set_level (PIN_LED_AMARILLO, 0);
      end if;

   end process_Standby_state;

   procedure process_Manual_state is
   begin

      -- Inicializar estado nuevo

      if prior_state /= current_state then
         prior_state := current_state;
         ada_esp_log (101);
         gpio_set_level (PIN_LED_VERDE, 1);
      end if;

      -- Proceso interno del estado

      if esp_timer_get_time >= t_1 + t_1_interval then

         t_1 := esp_timer_get_time;
         
         operate_servo_manual;
         mpu_operate (x_accel, y_accel, z_accel);
         operate_ina;

         vTaskDelay (1);

      end if;

      -- Revisar condiciones de transición

      --if state_change and button1_pressed then

      if state_change and button1_pressed and (gpio_get_level (PIN_BUTTON2)=1) then
         current_state := AUTOMATIC;
         state_change := false;
         button1_pressed := false;

      elsif button2_pressed then
         button2_pressed := False;
         case servo_state is
            when SERVO_FULL =>
               servo_state := SERVO_RESTRICTED;
               gpio_set_level (PIN_LED_ROJO, 1);

            when SERVO_RESTRICTED =>
               servo_state := SERVO_FULL;
               gpio_set_level (PIN_LED_ROJO, 0);

            when SERVO_REDUNDANT =>
               null;
         end case;
      end if;

      if not (gpio_get_level (PIN_BUTTON1) = 1) and not (gpio_get_level (PIN_BUTTON2) = 1) then
         current_state := STANDBY;
         state_change := false;
         button1_pressed := false;
         button2_pressed := false;
      end if;

      if prior_state /= current_state then
         servo_state := SERVO_FULL;
         gpio_set_level (PIN_LED_VERDE, 0);
         gpio_set_level (PIN_LED_ROJO, 0);
      end if;

   end process_Manual_state;

   procedure process_Automatic_state is

   begin

      -- Inicializar estado nuevo

      if prior_state /= current_state then
         prior_state := current_state;
         ada_esp_log (101);
         gpio_set_level (PIN_LED_VERDE, 1);
         gpio_set_level (PIN_LED_AMARILLO, 1);
      end if;

      -- Proceso interno del interno

      if esp_timer_get_time >= t_1 + t_1_interval then

         t_1 := esp_timer_get_time;
         
         mpu_operate (x_accel, y_accel, z_accel);
         operate_servo_auto;
         operate_ina;

         vTaskDelay (1);

      end if;

      -- Revisar condiciones de transición

      if state_change and button1_pressed and (gpio_get_level (PIN_BUTTON2) = 1) then
         current_state := MANUAL;
         state_change := false;
         button1_pressed := false;
         button2_pressed := false;

      elsif button2_pressed then
         button2_pressed := False;
      end if;

      if prior_state /= current_state then
         gpio_set_level (PIN_LED_VERDE, 0);
         gpio_set_level (PIN_LED_AMARILLO, 0);
      end if;

   end process_Automatic_state;

   -- FUNCIONES AUXILIARES DE INICIALIZACIÓN

   procedure ada_auxiliary_init is

   begin
      null;
   --for I in buffer_mobile_mean'Range loop
   --   buffer_mobile_mean (I) := Ada_Code.mobile_mean;
   --end loop;

   end ada_auxiliary_init;

   procedure ada_init_mpu is
   
   begin

      for i in 1 .. 3 loop
         tcaSelectChannel (mpu_get_channel(i));

         mpuWriteReg(16#6B#, 0);  -- MPU configuration
         mpuWriteReg(16#19#, 7);  -- sample rate 1KHz
         mpuWriteReg(16#1C#, 0);  -- ACC FS Range ±2g
         mpuWriteReg(16#1B#, 0);  -- GYR FS Range ±250º/s
         ada_esp_log (108);
         vTaskDelay (1);
      end loop;

   end ada_init_mpu;

   procedure ada_init_ina is

      data : constant UByte_buffer_t := (1 => 21, 2 => 111);

   begin

                      -- 0 001 010 1 = 21 : Canal3, 16 muestras, 2ms para bus
                      -- 01 101 111 = 111 : 2ms para shunt, modo continua shunt y bus
      inaWriteReg(16#00#, data);
      ada_esp_log (109);

   end ada_init_ina;

   -- FUNCIÓN PRINCIPAL DEL LOOP

   procedure ada_code_exec is

   begin

      case current_state is
         when STANDBY =>
            process_Standby_state;

         when MANUAL =>
            process_Manual_state;
            
         when AUTOMATIC =>
            process_Automatic_state;

         when others =>
            ada_esp_log (100);
      end case;
   end ada_code_exec;

begin

   -- INICIALIZACIÓN DE VARIABLES

   --null;

   t_1 := 0;
   t_1_interval := 20000;
   --t_2_interval := 1000000;
   --t_3_interval := 100000;

   mobile_mean := 1690;
   buffer_mobile_mean := (others => mobile_mean);
   counter := 50;

end Ada_Code;
