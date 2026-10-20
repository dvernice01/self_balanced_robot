#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// I2C dev address
#define MPU6050_I2C_ADDR       0xD0

// register's addresses
#define MPU6050_PWR_MGMT_1     0x6B
//#define MPU6050_FIFO_EN_REG    0x23
//#define MPU6050_USER_CTRL      0x6A

// configuration values for registers
#define MPU6050_WAKEUP_CLOCK   0x01
#define MPU6050_SLEEP_CLOCK    0x40
//#define MPU6050_FIFO_XG_ACCEL  0x48
//#define MPU6050_FIFO_ENABLE    0x40

#define ACC_RES                16384.0
#define GYRO_RES               131.0


typedef struct {
	float accel_x;
	float accel_y;
	float accel_z;
	float temp;
	float gyro_x;
	float gyro_y;
	float gyro_z;
} MPU6050_Data_t ;

void MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Start_DMA_Read (I2C_HandleTypeDef *hi2c);
void MPU6050_Process_Data(float *acc_x, float *acc_z, float *gyro_y);


#endif
