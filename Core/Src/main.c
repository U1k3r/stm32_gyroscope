/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay_micros.h"
#include "tm1637.h"
#include "MPU6050.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//int16_t Accel_X_RAW; //данные с акселерометра по оси X
int16_t Accel_Y_RAW;
int16_t Accel_Z_RAW;

int16_t Gyro_X_RAW; //данные с гироскопа по оси X


uint8_t Rec_Data[14]; //буфер хранения сырых данных

//uint8_t check = 0;

int32_t Angle_X = 0.0f;  // Текущий угол
int dec = 0;
int unit = 0;

int32_t alpha = 0.98;     // Коэффициент доверия гироскопу
uint32_t prev_time = HAL_GetTick();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  MPU6050_Init();

  DWT_Init();        // инициализация DWT
  set_brightness(7); // 0 - 7 яркость
  clearDisplay();    // очистка дисплея


  int8_t TimeDisp[4] = {0,};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
	   /*HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS_AD0_LOW,
	   MPU6050_RA_ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);

	   Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);

	   HAL_I2C_Mem_Read(&hi2c1,MPU6050_ADDRESS_AD0_LOW,
	   MPU6050_RA_GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
	   Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);*/


	   // 1. Измеряем время между итерациями (dt)
	       uint32_t current_time = HAL_GetTick();
	       float dt = (current_time - prev_time) / 1000.0f;  // в секундах
	       prev_time = current_time;

	       // 2. Читаем гироскоп и переводим в °/s
	       HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
	       Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	       float Gyro_X_DPS = Gyro_X_RAW / 131.0f;  // Для ±250°/s (FS_SEL=0)

	       // 3. Читаем акселерометр и вычисляем угол (в градусах)
	       HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);
	       //Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	       Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	       Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	       // Угол из акселерометра (atan2 в радианах, переводим в градусы)
	       float Accel_X_Angle = atan2(Accel_Y_RAW, Accel_Z_RAW) * 180.0f / M_PI;

	       // 4. Комбинируем данные фильтром
	       Angle_X = alpha * (Angle_X + Gyro_X_DPS * dt) + (1 - alpha) * Accel_X_Angle;

	       dec = (abs(Angle_X) % 90) / 10;  // Гарантирует результат в [0, 9)
	       unit = abs(Angle_X)%10; // ОШИБКА???

	   //вывод на экран

	   TimeDisp[0] = 0x7f;

	   if (Angle_X<0) {
	   	   TimeDisp[1] = 12;
	   }
	   else {
		   TimeDisp[1] = 0x7f;
	   }


	   TimeDisp[2] = dec;

	   TimeDisp[3] = unit;

	   display_mass(TimeDisp);

	   HAL_Delay(400);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
   }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
