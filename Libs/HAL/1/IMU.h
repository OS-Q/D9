/*
 * IMU.h
 *
 *  Created on: Jul 2, 2018
 *      Author: jeezus7596
 */

#ifndef IMU_H_
#define IMU_H_


//includes
#include "stm32f1xx_hal.h"
#include "math.h"


#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_TEMP_H       0x41
#define MPU6050_TEMP_L       0x42
#define ACCEL_XOUT			 0x3b
#define ACCEL_YOUT			 0x3d
#define ACCEL_ZOUT			 0x3f
#define TEMP_OUT			 0x41
#define GYRO_XOUT			 0X43
#define GYRO_YOUT			 0x45
#define GYRO_ZOUT			 0x47

#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG				0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define USER_CTRL		0x6A
#define INT_PIN_CFG  	0x37	//
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I			0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	SlaveAddress	0xD0	//IIC写入时的地址字节数据



#define PI					 3.1415



extern I2C_HandleTypeDef hi2c1;

/*
 * Internal functions
 */
HAL_StatusTypeDef readMPU6050(uint8_t reg, uint8_t *data);
HAL_StatusTypeDef writeMPU6050(uint8_t reg, uint8_t data);

/*
 * IO Function prototypes
 */
HAL_StatusTypeDef MPU6050Init();
void readAccel(float *AccX, float *AccY, float *AccZ);
void readGyro(float *GyroX, float *GyroY, float *GyroZ);
void getTemperature(float *temperature);
void getRoll (float *Roll);
void getPitch (float *Pitch);
void getYaw(float* Yaw);
void getYPR(float* Yaw, float* Pitch, float* Roll);



#endif /* IMU_H_ */
