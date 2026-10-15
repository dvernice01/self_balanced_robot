#include "mpu6050.h"

uint8_t mpu_buffer[12]; // gyro_z is not considered because not used

//void MPU6050_Init(I2C_HandleTypeDef *hi2c) {
//    // Wake up of the sensor
//    uint8_t data_pwr = 0x00;
//    HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data_pwr, 1, 100);
//    HAL_Delay(200);
//}

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    uint8_t data_pwr = 0x00;

    // Check for who_am_i register
    HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, 0x75, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);

    // 0x68 is the default value
//    if (check == 0x68) {
//        HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, 0x6B, I2C_MEMADD_SIZE_8BIT, &data_pwr, 1, 100);
//        return 1;
//    }

    return 1;
}

void MPU6050_Start_DMA_Read(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_I2C_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, mpu_buffer, 12);
}

void MPU6050_Process_Data(int16_t *acc_x, int16_t *acc_z, int16_t *gyro_y) {

	// Data have 16bits. We are just using lin acc on x and z and ang acc on y
    *acc_x = (int16_t)((mpu_buffer[0] << 8) | mpu_buffer[1]);

    *acc_z = (int16_t)((mpu_buffer[4] << 8) | mpu_buffer[5]);

    *gyro_y = (int16_t)((mpu_buffer[10] << 8) | mpu_buffer[11]);
}
