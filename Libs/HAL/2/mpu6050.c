/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"


/**
* @brief  init
*         Checks if the mpu6050 is connected and power it on.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @retval if there is some error in the initialization.
*/
int init(I2C_HandleTypeDef *i2c){
	uint8_t temp;
	uint8_t address = (uint8_t)MPU_ADDRESS;
	uint8_t who = (uint8_t)WHO_AM_I;
	unsigned char buffer[2];

	buffer[0] = (uint8_t)PWR_MGMT_1; 
	buffer[1] = 0x00;

	//Check if the MPU is connected
	if(HAL_I2C_Master_Transmit(i2c,address,&who,1,100)!=HAL_OK) return 1;
	if(HAL_I2C_Master_Receive(i2c,address,&temp,1,100) != HAL_OK){ return 2; }
	else{
		if(temp == 0x68){
			//Power On the MPU writing a 0 in the PWR_MGMT_1 register
			if(HAL_I2C_Master_Transmit(i2c,address,buffer,2,100)!=HAL_OK) return 1;
		}
		else return 3;
	}

	return 0;
}

/**
* @brief  Gyroscope Configuration
*         Trigger gyroscope self-test and configure the gyroscope's full scale range.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  range: variable that set the scale range from 250º/s to 2000º/s.
* @retval if there is some error in the i2c connection.
*/
int gyro_configuration(I2C_HandleTypeDef *i2c, int range){

	uint8_t range_hex, address = (uint8_t)MPU_ADDRESS;
	unsigned char buffer[2];

	switch(range){
		case 250:
			range_hex = 0x00;
			break;
		case 500:
			range_hex = 0x08;
			break;
		case 1000:
			range_hex = 0x10;
			break;
		case 2000:
			range_hex = 0x18;
			break;
	}

	buffer[0] = (uint8_t)GYRO_CONFIG; 
	buffer[1] = range_hex;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,2,100)!=HAL_OK) return 1;
	return 0;
}

/**
* @brief  Accelerometer configuration
*         Trigger accelerometer self test and configure the accelerometer full scale range.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  range: variable that set the scale range from 2G to 16G.
* @retval if there is some error in the i2c connection.
*/
int accel_configuration(I2C_HandleTypeDef *i2c,int range){

	uint8_t range_hex, address = (uint8_t)MPU_ADDRESS;
	unsigned char buffer[2];

	switch(range){
		case 2:
			range_hex = 0x00;
			break;
		case 4:
			range_hex = 0x08;
			break;
		case 8:
			range_hex = 0x10;
			break;
		case 16:
			range_hex = 0x18;
			break;
	}

	buffer[0] = (uint8_t)ACCEL_CONFIG; 
	buffer[1] = range_hex;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,2,100)!=HAL_OK) return 1;
	return 0;
}


/**
* @brief  Gyroscope calibration
*         Set the default values from the gyroscope when it is in a standstill position.
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  mpu: struct that stores all the data from the mpu6050.
* @retval if there is some error in the i2c connection.
*/
int calibration(I2C_HandleTypeDef *i2c, mpu6050 * mpu){
	int i = 0;
	for (i = 0; i < 5000; ++i){
		read_gyro(i2c,mpu);
		mpu->gyro_x_cal += mpu->gyro_x;
		mpu->gyro_y_cal += mpu->gyro_y;
		mpu->gyro_z_cal += mpu->gyro_z;
	}
	mpu->gyro_x_cal /= 5000;
	mpu->gyro_y_cal /= 5000;
	mpu->gyro_z_cal /= 5000;

	return 0;
}

/**
* @brief  Read temperature
*         Saves the temperature read from the mpu6050
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  mpu: struct that stores all the data from the mpu6050.
* @retval if there is some error in the i2c connection.
*/
int read_temperature(I2C_HandleTypeDef *i2c, mpu6050 * mpu){
	uint8_t address = (uint8_t)MPU_ADDRESS;
	unsigned char buffer[2];
	uint16_t temp;

	buffer[0] = (uint8_t)TEMP_OUT_H;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,1,100)!=HAL_OK) return 1;
	if(HAL_I2C_Master_Receive(i2c,address,buffer,2,100) != HAL_OK){ return 2; }

	temp = buffer[0]<<8 | buffer[1];
	mpu->temperature  = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	return 0;
}

/**
* @brief  Read Gyroscope values
*         Read the gyroscope instant values from the mpu6050
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  mpu: struct that stores all the data from the mpu6050.
* @retval if there is some error in the i2c connection.
*/
int read_gyro(I2C_HandleTypeDef *i2c, mpu6050 * mpu){
	uint8_t address = (uint8_t)MPU_ADDRESS;
	unsigned char buffer[6];

	buffer[0] = (uint8_t)GYRO_XOUT_H;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,1,100)!=HAL_OK){ return 1; }
	if(HAL_I2C_Master_Receive(i2c,address,buffer,6,100) != HAL_OK){ return 2; }

	mpu->gyro_x = (int16_t)(buffer[0]<<8 | buffer[1]);
	mpu->gyro_y = (int16_t)(buffer[2]<<8 | buffer[3]);
	mpu->gyro_z = (int16_t)(buffer[4]<<8 | buffer[5]);

	return 0;

}

/**
* @brief  Read Accelerometer values
*         Read the accelerometer instant values from the mpu6050
* @param  i2c_var: variable that is the i2c handler for HAL libray.
* @param  mpu: struct that stores all the data from the mpu6050.
* @retval if there is some error in the i2c connection.
*/
int read_acc(I2C_HandleTypeDef *i2c, mpu6050 * mpu){
	uint8_t address = (uint8_t)MPU_ADDRESS;
	unsigned char buffer[6];

	buffer[0] = (uint8_t)ACCEL_XOUT_H;

	if(HAL_I2C_Master_Transmit(i2c,address,buffer,1,100)!=HAL_OK){ return 1; }
	if(HAL_I2C_Master_Receive(i2c,address,buffer,6,100) != HAL_OK){ return 2; }

	mpu->accel_x = (int16_t)(buffer[0]<<8 | buffer[1]);
	mpu->accel_y = (int16_t)(buffer[2]<<8 | buffer[3]);
	mpu->accel_z = (int16_t)(buffer[4]<<8 | buffer[5]);

	return 0;
}
