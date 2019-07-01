/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  Pedro Torres
  * @version V1.0
  * @date    30-January-2018
  * @brief   Header file for mpu6050.c file
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MPU6050_H
#define MPU6050_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Registers of MPU6050 */
/**
  * @brief  MPU6050 configuration registers 
  */
#define MPU_ADDRESS		0x68<<1
#define PWR_MGMT_1 		0x6B
#define GYRO_CONFIG 	0x1B
#define ACCEL_CONFIG 	0x1C
#define WHO_AM_I		0x75

/**
  * @brief  MPU6050 accelerometer registers 
  */
#define ACCEL_XOUT_H	0x3B
#define ACCEL_XOUT_L	0x3C
#define ACCEL_YOUT_H	0x3D
#define ACCEL_YOUT_L	0x3E
#define ACCEL_ZOUT_H	0x3F
#define ACCEL_ZOUT_L	0x40

/**
  * @brief  MPU6050 gyroscope registers 
  */
#define GYRO_XOUT_H		0x43
#define GYRO_XOUT_L		0x44
#define GYRO_YOUT_H		0x45
#define GYRO_YOUT_L		0x46
#define GYRO_ZOUT_H		0x47
#define GYRO_ZOUT_L		0x48

/**
  * @brief  MPU6050 temperature registers 
  */
#define TEMP_OUT_H		0x41
#define TEMP_OUT_L		0x42

/* Private variables ---------------------------------------------------------*/
/**
* @brief Private struct variable to store all data from the MPU6050.
*/
struct MPU_var{
    int16_t gyro_x_cal;
    int16_t gyro_y_cal;
    int16_t gyro_z_cal;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    float temperature;
};

typedef struct MPU_var mpu6050;

/** @defgroup MPU6050_FunctionsPrototype
  * @{
  */
int init(I2C_HandleTypeDef *i2c);
int gyro_configuration(I2C_HandleTypeDef *i2c,int range);
int accel_configuration(I2C_HandleTypeDef *i2c,int range);
int calibration(I2C_HandleTypeDef *i2c, mpu6050 * mpu);
int read_temperature(I2C_HandleTypeDef *i2c, mpu6050 * mpu);
int read_gyro(I2C_HandleTypeDef *i2c, mpu6050 * mpu);
int read_acc(I2C_HandleTypeDef *i2c, mpu6050 * mpu);
/**
  * @}
  */

#endif
