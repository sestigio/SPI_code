#ifndef _ADA_HAL_H_
#define _ADA_HAL_H_

/* API exported to Ada/SPARK */

void __gnat_last_chance_handler (char *source_location,
    int line);


/* API imported from Ada/SPARK */

extern int current_state;
extern int prior_state;
extern int servo_state;
extern int gyro_state;

extern int mobile_mean;
extern int servo_bearing;

extern float x_accel;
extern float y_accel;
extern float z_accel;

//extern void mpuReadfromReg(uint8_t mpu_register, uint8_t *read_buffer, uint8_t length);

//extern void mpuWriteReg(uint8_t mpu_register, uint8_t write_buffer);

//extern void inaReadfromReg(uint8_t mpu_register, uint8_t *read_buffer, uint8_t length);

//extern void inaWriteReg(uint8_t mpu_register, uint8_t *write_buffer, uint8_t length);

extern void Ada_Codeinit(void);

extern void ada_auxiliary_init(void);

extern void ada_init_mpu(void);

extern void ada_init_ina(void);

extern void ada_code_exec(void);

#endif /* ! _ADA_HAL_H_ */