/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <string.h>
#include <stdbool.h>
#include "i2c-lcd.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
UART_HandleTypeDef huart1;
/* USER CODE BEGIN PV */
uint8_t tx_buffer[30] = "Radios are a pain sometimes\n\r";
uint8_t tx_buffer1[22] = "This Project is fun!!!";
uint8_t tx_buffer2[8] = "Sampling";
uint8_t rx_indx;
uint8_t rx_data[100];
uint8_t rx_buffer[100];
uint8_t transfer_cplt;
uint16_t pulse_time = 500;
uint16_t wait_time = 2500;
char audioToPlay[] = "green";
bool pending_play = false;
char red[] = "red";
char green[] = "green";
char purple[] = "purple";
char blue[] = "blue";
char orange[] = "orange";
char yellow[] = "yellow";
char black[] = "black";
char white[] = "white";
char pink[] = "pink";
char grey[] = "grey";
char brown[] = "brown";
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
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
  HAL_UART_Receive_IT(&huart1, rx_data,1);
  lcd_init();
  lcd_clear();
  lcd_put_cur(0,0);
  lcd_send_string("Welcome to");
//  HAL_Delay(1000);
  lcd_put_cur(1,0);
  lcd_send_string("ColourAid 1.0");
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(pending_play == true)
	  {
		 display_info(audioToPlay);
		 play_audio(audioToPlay);
		 pending_play = false;
	  }
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
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */
  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10, GPIO_PIN_SET);
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : PC0 PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pins : PA6 PA7 PA8 PA9
                           PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char colour[] = "green";
	UNUSED(huart);
	uint8_t i;
		if(huart->Instance == USART1)
		{
			if (rx_indx==0) {
				for(i=0; i<100;i++)
					rx_buffer[i]=0; //Clearing buffer IMPORTANT!
//				HAL_UART_Transmit(&huart1, tx_buffer1, 22, 100);
			}
			if (rx_data[0] != 13){ //13 is the character for enter
				rx_buffer[rx_indx++] = rx_data[0];
				//if not enter, then buffer at next index is assigned rx_data[0]
			}
			else{
				rx_indx = 0;
				transfer_cplt = 1;
//				HAL_UART_Transmit(&huart1, "\n\r", 2, 100);
				if (!strcmp(rx_buffer, "red"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "red");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "blue"))
					{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "blue");
					strcpy(audioToPlay, colour);
					pending_play = true;
					}
				else if (!strcmp(rx_buffer, "green"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "green");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "purple"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "purple");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "orange"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "orange");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "yellow"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "yellow");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "black"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "black");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "white"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "white");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "pink"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "pink");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "grey"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "grey");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
				else if (!strcmp(rx_buffer, "brown"))
				{
//					HAL_UART_Transmit(&huart1, tx_buffer2, 8, 100);
					strcpy(colour, "brown");
					strcpy(audioToPlay, colour);
					pending_play = true;
				}
			}
			HAL_UART_Receive_IT(&huart1, rx_data, 1);
//			HAL_UART_Transmit(&huart1, rx_data, strlen(rx_data), 100);
		}
//	HAL_UART_Transmit(&huart1, rx_data, 1, 100);
//	HAL_UART_Transmit(&huart1, tx_buffer, 30, 100);
}
void play_audio (char str[])
  {
	  char red[] = "red";
	  char green[] = "green";
	  char purple[] = "purple";
	  char blue[] = "blue";
	  char orange[] = "orange";
	  char yellow[] = "yellow";
	  char black[] = "black";
	  char white[] = "white";
	  char pink[] = "pink";
	  char grey[] = "grey";
	  char brown[] = "brown";
	  if (strncmp(str, red, strlen(str)) == 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if(strncmp(str, green, strlen(str)) == 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if(strncmp(str, purple, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, blue, strlen(str)) == 0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, orange, strlen(str)) == 0)
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, yellow, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if(strncmp(str, black, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, white, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, pink, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, grey, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else if (strncmp(str, brown, strlen(str)) == 0) {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
		  HAL_Delay(pulse_time);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
		  HAL_Delay(1000);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		  HAL_Delay(wait_time);
	  }
  }
void display_info(char str[])
{
	lcd_clear();
	lcd_put_cur(0,0);
	if (strncmp(str, red, strlen(str)) == 0)
	{
		lcd_send_string("Red");
		lcd_put_cur(1,0);
		lcd_send_string("R255 G0 B0");
	}
	else if (strncmp(str, green, strlen(str)) == 0)
	{
		lcd_send_string("Green");
		lcd_put_cur(1,0);
		lcd_send_string("R0 G255 B0");
	}
	else if (strncmp(str, purple, strlen(str)) == 0)
	{
		lcd_send_string("Purple");
		lcd_put_cur(1,0);
		lcd_send_string("R157 G0 B255");
	}
	else if (strncmp(str, blue, strlen(str)) == 0)
	{
		lcd_send_string("Blue");
		lcd_put_cur(1,0);
		lcd_send_string("R0 G0 B255");
	}
	else if (strncmp(str, orange, strlen(str)) == 0)
		{
			lcd_send_string("Orange");
			lcd_put_cur(1,0);
			lcd_send_string("R255 G165 B0");
		}
	else if (strncmp(str, yellow, strlen(str)) == 0)
		{
			lcd_send_string("Yellow");
			lcd_put_cur(1,0);
			lcd_send_string("R255 G222 B33");
		}
	else if (strncmp(str, black, strlen(str)) == 0)
		{
			lcd_send_string("Black");
			lcd_put_cur(1,0);
			lcd_send_string("R0 G0 B0");
		}
	else if (strncmp(str, white, strlen(str)) == 0)
		{
			lcd_send_string("White");
			lcd_put_cur(1,0);
			lcd_send_string("R255 G255 B255");
		}
	else if (strncmp(str, pink, strlen(str)) == 0)
		{
			lcd_send_string("Pink");
			lcd_put_cur(1,0);
			lcd_send_string("R255 G141 B161");
		}
	else if (strncmp(str, grey, strlen(str)) == 0)
		{
			lcd_send_string("Grey");
			lcd_put_cur(1,0);
			lcd_send_string("R128 G128 B128");
		}
	else if (strncmp(str, brown, strlen(str)) == 0)
		{
			lcd_send_string("Brown");
			lcd_put_cur(1,0);
			lcd_send_string("R150 G75 B0");
		}
}
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
