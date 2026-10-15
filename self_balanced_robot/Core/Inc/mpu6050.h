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
//#define MPU6050_FIFO_XG_ACCEL  0x48
//#define MPU6050_FIFO_ENABLE    0x40


typedef struct {
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} MPU6050_Data_t ;

//void MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Start_DMA_Read (I2C_HandleTypeDef *hi2c);
void MPU6050_Process_Data(int16_t *acc_x, int16_t *acc_z, int16_t *gyro_y);

// Esempio: uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
// Esempio: void MPU6050_Read_DMA(...);

#endif
