#include "MPU6050.h"
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"


void MPU6050_Init(void)
{
	uint8_t check=0;
	uint8_t Data = 0;
	HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS_AD0_LOW,
			MPU6050_RA_WHO_AM_I,1,&check,1,1000);

	if (check == 104)// если считали данные из регистра
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS_AD0_LOW,
				MPU6050_RA_PWR_MGMT_1, 1,&Data, 1, 1000);
		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS_AD0_LOW,
				MPU6050_RA_SMPLRT_DIV, 1, &Data, 1, 1000);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS_AD0_LOW,
				MPU6050_RA_ACCEL_CONFIG, 1, &Data, 1, 1000);
		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 +-/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS_AD0_LOW,
				MPU6050_RA_GYRO_CONFIG, 1, &Data, 1, 1000);
		uint8_t str[30]={0};//инициализируем буфер-массив
		sprintf(str,"MPU6050 is ready!\r\n");//преобразуем массив в строку
//		HAL_UART_Transmit(&huart1, str, 18, 100);//выводим строку по UART
	}

	else
	{
		uint8_t str[30]={0};
		sprintf(str,"We have a MPU6050 problem..\r\n");
//		HAL_UART_Transmit(&huart1, str, 27, 100);
	}
}
