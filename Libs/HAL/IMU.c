/*
 * IMU.c
 *
 *  Created on: Jul 2, 2018
 *      Author: jeezus7596
 */
#include "IMU.h"
#include "stm32f1xx_hal.h"


HAL_StatusTypeDef writeMPU6050(uint8_t reg, uint8_t data)
{
	return HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR<<1, reg, I2C_MEMADD_SIZE_8BIT, &data, I2C_MEMADD_SIZE_8BIT, 100);
}


HAL_StatusTypeDef readMPU6050(uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR<<1, reg, I2C_MEMADD_SIZE_8BIT, data, I2C_MEMADD_SIZE_8BIT, 100);
}

/**
  * @brief  Initializes MPU6050 Inertial Measurement Unit
  * @retval HAL status
  */

HAL_StatusTypeDef MPU6050Init()
{
	if (HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADDR<<1, 5, 100) != HAL_OK)
		return HAL_I2C_STATE_ERROR;

//	writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
//	writeMPU6050(MPU6050_CONFIG, 0x00);
//	writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);
//	writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
//	writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
	
	writeMPU6050(PWR_MGMT_1,0x00);                //½â³ýÐÝÃß×´Ì¬
	writeMPU6050(INT_PIN_CFG, 0x02);
	writeMPU6050(SMPLRT_DIV,0x07);
	writeMPU6050(USER_CTRL, 0x07);
	writeMPU6050(CONFIG,0x06);
	writeMPU6050(GYRO_CONFIG,0x18);
	writeMPU6050(ACCEL_CONFIG,0x01);
	return HAL_OK;

}

/**
 * @brief Writes accelerometer values to variables pointed by the parameters
 * @param x axis value
 * @param y axis value
 * @param z axis value
 * @retval NULL
 */

void readAccel(float *AccX, float *AccY, float *AccZ)
{
	static uint8_t tempXL, tempXH, tempYL, tempYH, tempZL, tempZH;
	static int16_t tempX, tempY, tempZ;
	readMPU6050(ACCEL_XOUT, &tempXH);
	readMPU6050(ACCEL_XOUT+0x01, &tempXL);
	readMPU6050(ACCEL_YOUT, &tempYH);
	readMPU6050(ACCEL_YOUT+0x01, &tempYL);
	readMPU6050(ACCEL_ZOUT, &tempZH);
	readMPU6050(ACCEL_ZOUT+0x01, &tempZL);

	tempX = ((tempXH<<8) | (tempXL));
	tempY = ((tempYH<<8) | (tempYL));
	tempZ = ((tempZH<<8) | (tempZL));

	*AccX = (float)tempX/16384;
	*AccY = (float)tempY/16384;
	*AccZ = (float)tempZ/16384;

}



/**
 * @brief Writes gyroscope values to variables pointed by the parameters
 * @param x axis value
 * @param y axis value
 * @param z axis value
 * @retval NULL
 */

void readGyro(float *AccX, float *AccY, float *AccZ)
{
	static uint8_t tempXL, tempXH, tempYL, tempYH, tempZL, tempZH;
	static int16_t tempX, tempY, tempZ;
	readMPU6050(GYRO_XOUT, &tempXH);
	readMPU6050(GYRO_XOUT+0x01, &tempXL);
	readMPU6050(GYRO_YOUT, &tempYH);
	readMPU6050(GYRO_YOUT+0x01, &tempYL);
	readMPU6050(GYRO_ZOUT, &tempZH);
	readMPU6050(GYRO_ZOUT+0x01, &tempZL);

	tempX = ((tempXH<<8) | (tempXL));
	tempY = ((tempYH<<8) | (tempYL));
	tempZ = ((tempZH<<8) | (tempZL));

	*AccX = (float)tempX/65.5;
	*AccY = (float)tempY/65.5;
	*AccZ = (float)tempZ/65.5;

}


/**
 * @brief Writes Yaw, pitch and roll value to variables pointed by the function arguments
 * @param Yaw in degrees
 * @param Pitch in degrees
 * @param Roll in degrees
 * @retval NULL
 */
void getYPR(float* Yaw, float* Pitch, float* Roll)
{
	static float accX, accY, accZ, gyroX, gyroY, gyroZ;
	static float gyroYaw = 0, gyroPitch = 0, gyroRoll = 0, time, prevTime = 0;

	readAccel(&accX, &accY, &accZ);
	readGyro(&gyroX, &gyroY, &gyroZ);

	time = (HAL_GetTick() - prevTime)*0.001;
	gyroYaw = gyroYaw + gyroZ*time;
	gyroPitch = gyroPitch + gyroY*time;
	gyroRoll = gyroRoll + gyroY*time;
	prevTime = HAL_GetTick();

	*Roll = gyroRoll*0.01 + (atan2(accY, accZ + fabsf(accX)) * 360 / 2.0 / PI)*0.99;
	*Yaw = gyroYaw;
	*Pitch = gyroPitch * 0.01 + (atan2(accX, accZ + fabsf(accY)) * 360 / (-2.0) / PI) * 0.99;
}




/**
 * @brief Writes temperature value to variable pointed by the parameter
 * @param temperature axis value
 * @retval NULL
 */
void getTemperature(float *temperature)
{
	static uint8_t tempXL, tempXH;
	static int16_t tempX;
	readMPU6050(TEMP_OUT, &tempXH);
	readMPU6050(TEMP_OUT+0x01, &tempXL);


	tempX = ((tempXH<<8) | (tempXL));

	*temperature = (float)tempX/340 + 36.53;

}


/**
 * @brief Writes Roll value to variable pointed by the parameter
 * @param Roll in degrees
 * @retval NULL
 */
void getRoll (float *Roll)
{
	static float accX, accY, accZ, gyroX, gyroY, gyroZ;
	static float gyroRoll = 0, time, prevTime = 0;

	readAccel(&accX, &accY, &accZ);
	readGyro(&gyroX, &gyroY, &gyroZ);

	time = (HAL_GetTick() - prevTime)*0.001;
	gyroRoll = gyroRoll + gyroX*time;
	prevTime = HAL_GetTick();

	*Roll = gyroRoll*0.01 + (atan2(accY, accZ + fabsf(accX)) * 360 / 2.0 / PI)*0.99;
}

/**
 * @brief Writes pitch value to variable pointed by the parameter
 * @param Roll in degrees
 * @retval NULL
 */
void getPitch (float *Pitch)
{
	static float accX, accY, accZ, gyroX, gyroY, gyroZ;
	static float gyroPitch = 0, time, prevTime = 0;

	readAccel(&accX, &accY, &accZ);
	readGyro(&gyroX, &gyroY, &gyroZ);

	time = (HAL_GetTick() - prevTime)*0.001;
	gyroPitch = gyroPitch + gyroY*time;
	prevTime = HAL_GetTick();

	*Pitch = gyroPitch * 0.01 + (atan2(accX, accZ + fabsf(accY)) * 360 / (-2.0) / PI) * 0.99;

}


/**
 * @brief Writes Yaw value to variable pointed by the parameter by integrating gyroZ
 * @param Yaw in degrees
 * @retval NULL
 */
void getYaw(float* Yaw)
{
	static float gyroX, gyroY, gyroZ;
	static float gyroYaw = 0, time, prevTime = 0;

	readGyro(&gyroX, &gyroY, &gyroZ);

	time = (HAL_GetTick() - prevTime)*0.001;
	gyroYaw = gyroYaw + gyroZ*time;
	prevTime = HAL_GetTick();

	*Yaw = gyroYaw;

}



