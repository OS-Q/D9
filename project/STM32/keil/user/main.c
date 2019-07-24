#include "stm32f10x.h"
#include "I2C_MPU6050.h"
#include "usart1.h"
#include "delay.h"
#include "stdio.h"

int fputc(int ch,FILE *f)//printf重载
{
	USART_SendData(USART1,(u8)ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	return ch;
}

int main(void)
{	
	USART1_Config();	 //串口初始化
	printf("begin\r\n");
	I2C_MPU6050_Init(); 	//IIC接口初始化
  InitMPU6050();   	
	while(1)
	{
		printf("x %8d   ",GetData(ACCEL_XOUT_H));   //加速度X轴原始数据
		printf("y %8d   ",GetData(ACCEL_YOUT_H));   //加速度Y轴原始数据
		printf("z %8d   ",GetData(ACCEL_ZOUT_H));   //加速度Z轴原始数据
		
		printf("%8d   ",GetData(GYRO_XOUT_H));    //陀螺仪X轴原始数据
		printf("%8d   ",GetData(GYRO_YOUT_H));    //陀螺仪Y轴原始数据
		printf("%8d   ",GetData(GYRO_ZOUT_H));    //陀螺仪Z轴原始数据
		printf("temp %8f   ",GetData(TEMP_OUT_H)/340.0+35.0);    //温度
		printf("WHO_AM_I %8d   ",I2C_ByteRead(WHO_AM_I)); 
		printf("\n");
		delay_ms(1000);
	}		

 
}
