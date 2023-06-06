/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "i2c-lcd.c"
#include "fonts.h"
#include "ssd1306.h"
#include<stdio.h>
#include<math.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MPU6050_ADDR 								0xD0
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_WHO_AM_I         0x75
#define g 													9.81
char buff[5];
float Ax, Ay, Az, Gx, Gy, Gz, Acc, prevAcc = 10.5;
int step = 0;
int state = 0;
uint32_t blinkTimer = 0;
uint32_t blinkDelay = 500;
uint32_t mpuTimer = 0;
uint32_t mpuDelay = 500;
// Define the button state
typedef enum {
  BUTTON_RELEASED,
  BUTTON_PRESSED
} ButtonState;

// Global variables
ButtonState buttonState = BUTTON_RELEASED;
uint8_t paused = 0;


void MPU6050_Init(void)
{
	uint8_t check, data;
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_WHO_AM_I, 1, &check, 1, 1000);
	if (check == 104)
	{
		data = 0; 
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 1, &data, 1, 1000);
		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 1, &data, 1, 1000);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 1, &data, 1, 1000);
		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 1, &data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	int16_t Accel_X_Raw, Accel_Y_Raw, Accel_Z_Raw;
	//int16_t Ax, Ay, Az;
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H, 1, Rec_Data, 6, 1000);
	Accel_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Ax = Accel_X_Raw*g/16384.0;
	Ay = Accel_Y_Raw*g/16384.0;
	Az = Accel_Z_Raw*g/16384.0;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];
	int16_t Gyro_X_Raw, Gyro_Y_Raw, Gyro_Z_Raw;
	//int16_t Gx, Gy, Gz;
	
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H, 1, Rec_Data, 6, 1000);
	Gyro_X_Raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_Raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_Raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
	Gx = Gyro_X_Raw/131.0;
	Gy = Gyro_Y_Raw/131.0;
	Gz = Gyro_Z_Raw/131.0;
}

void Button_Handler(void) {
  if (paused == 0) {
    // Pause the system
    paused = 1;
    SSD1306_GotoXY (65, 51);
		SSD1306_Puts("Paused ", &Font_7x10, 1);
		SSD1306_UpdateScreen();
  } else {
    // Unpause the system
    paused = 0;
    SSD1306_GotoXY (65, 51);
		SSD1306_Puts("Running", &Font_7x10, 1);
		SSD1306_UpdateScreen();
  }
}
// Button interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == Pause_Button_Pin) {
    Button_Handler();
  }
}

// Button interrupt handler
void EXTI0_IRQHandler(void) {
  HAL_GPIO_EXTI_IRQHandler(Pause_Button_Pin);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
	SSD1306_Init();
  SSD1306_GotoXY (13,23);
  SSD1306_Puts ("Loading", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	HAL_Delay (500);
	SSD1306_Puts (".", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	HAL_Delay (500);
	SSD1306_Puts (".", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	HAL_Delay (500);
	SSD1306_Puts (".", &Font_11x18, 1);
	SSD1306_UpdateScreen();
	HAL_Delay (500);
  SSD1306_Clear();
	SSD1306_GotoXY (0, 0);
	SSD1306_Puts ("Acc", &Font_7x10, 1);
	SSD1306_GotoXY (65, 0);
	SSD1306_Puts ("Details", &Font_7x10, 1);
	SSD1306_UpdateScreen();
	int button_check = 0;
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		if (HAL_GetTick() - blinkTimer >= blinkDelay)
    {
      // Toggle the LED state
      HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);

      // Update the blink timer
      blinkTimer = HAL_GetTick();
    }
    if (HAL_GPIO_ReadPin(GPIOB, Pause_Button_Pin) == 1) {
			if (buttonState == BUTTON_RELEASED) {
					buttonState = BUTTON_PRESSED;
					Button_Handler();
				}
    }	 else {
      if (buttonState == BUTTON_PRESSED) {
        buttonState = BUTTON_RELEASED;
      }
    }
		if (HAL_GPIO_ReadPin(Reset_Button_GPIO_Port,Reset_Button_Pin) == 0) {
			step = 0;
		}
		if (HAL_GetTick() - mpuTimer >= mpuDelay) {
		MPU6050_Read_Accel();
		MPU6050_Read_Gyro();
		Acc = sqrt(Ax*Ax + Ay*Ay + Az*Az);
		sprintf(buff, "x:%.2f", Ax);
		SSD1306_GotoXY (0, 17);
		SSD1306_Puts (buff, &Font_7x10, 1);
		sprintf(buff, "y:%.2f", Ay);
		SSD1306_GotoXY (0, 34);
		SSD1306_Puts (buff, &Font_7x10, 1);
		sprintf(buff, "z:%.2f", Az);
		SSD1306_GotoXY (0, 51);
		SSD1306_Puts (buff, &Font_7x10, 1);
		sprintf(buff, "Acc:%.2f", Acc);
		SSD1306_GotoXY (65, 17);
		SSD1306_Puts (buff, &Font_7x10, 1);
		if ((Acc - prevAcc > 8) || (prevAcc - Acc > 8)) 
		{	
			if (paused == 0) {
			step++;
			}
		}
		sprintf(buff, "step:%d", step);
		SSD1306_GotoXY (65, 34);
		SSD1306_Puts (buff, &Font_7x10, 1);
		
		SSD1306_UpdateScreen();
		mpuTimer = HAL_GetTick();
		}
		//HAL_Delay (100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		prevAcc = Acc;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Blue_GPIO_Port, LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Blue_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Reset_Button_Pin Pause_Button_Pin */
  GPIO_InitStruct.Pin = Reset_Button_Pin|Pause_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
/* USER CODE BEGIN MX_GPIO_Init_2 */
	
/* USER CODE END MX_GPIO_Init_2 */
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
