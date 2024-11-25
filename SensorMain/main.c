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
#include "stdbool.h"
#include "string.h"
#include <stdio.h>
#include <math.h>

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
/
struct HSV
{
  double Hue, Saturation, Value;
  char c_name[20];
};

uint8_t tx_red[4] = "red\r";
uint8_t tx_blue[5] = "blue\r";
uint8_t tx_green[6] = "green\r";
uint8_t tx_purple[7] = "purple\r";
uint8_t tx_orange[7] = "orange\r";
uint8_t tx_yellow[7] = "yellow\r";
uint8_t tx_black[6] = "black\r";
uint8_t tx_white[6] = "white\r";
uint8_t tx_pink[5] = "pink\r";
uint8_t tx_grey[5] = "grey\r";
uint8_t tx_brown[6] = "brown\r";

uint8_t rx_indx;
uint8_t rx_data[100];
uint8_t rx_buffer[100];
uint8_t transfer_cplt;


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
#define TCS34725_ADDRESS          (0x29 << 1) /* I2C address */
#define TCS34725_COMMAND_BIT      (0x80)      /* Command bit */
#define TCS34725_ENABLE           (0x00)      /* Enable register */
#define TCS34725_ENABLE_AEN       (0x02)      /* RGBC Enable */
#define TCS34725_ENABLE_PON       (0x01)      /* Power on */
#define TCS34725_ATIME            (0x01)      /* Integration time */
#define TCS34725_CONTROL          (0x0F)      /* Set the gain level */
#define TCS34725_ID               (0x12)
#define TCS34725_CDATAL           (0x14)      /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)      /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)      /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)      /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)
#define TCS34725_INTEGRATIONTIME_50MS   0xEB  /* 50ms  - 20 cycles */
#define TCS34725_GAIN_4X                0x01  /* 4x gain  */
uint8_t _tcs34725Initialised = 0;
int red, green, blue;

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

  getRGB(&red, &green, &blue);

 	float hue(int a, int b, int c);
  float saturation(int a, int b, int c);
  float value(int a, int b, int c);
  char* hsv_string(struct HSV hsv_c);

  float currentHue = hue(red,green,blue);
  float currentSat = saturation(red,green,blue);
  float currentVal = value(red,green,blue);
  struct HSV HSV_colour = {currentHue, currentSat, currentVal, "none"};
  sendColour(hsv_string(HSV_colour));


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void sendColour (char pass_colour[])
{
	char colour[] = "green";


	strcpy(colour, pass_colour);

	if (!strcmp(colour, "red"))
	{
	HAL_UART_Transmit(&huart1, tx_red,4, 100);
	}
	else if (!strcmp(colour, "blue"))
	{
	HAL_UART_Transmit(&huart1, tx_blue,5, 100);
	}
	else if (!strcmp(colour, "green"))
	{
		HAL_UART_Transmit(&huart1, tx_green,6, 100);
	}
	else if (!strcmp(colour, "purple"))
	{
		HAL_UART_Transmit(&huart1, tx_purple,7, 100);
	}
	else if (!strcmp(colour, "orange"))
	{
		HAL_UART_Transmit(&huart1, tx_orange,7, 100);
	}
	else if (!strcmp(colour, "yellow"))
	{
		HAL_UART_Transmit(&huart1, tx_yellow,7, 100);
	}
	else if (!strcmp(colour, "black"))
	{
		HAL_UART_Transmit(&huart1, tx_black,6, 100);
	}
	else if (!strcmp(colour, "white"))
	{
	HAL_UART_Transmit(&huart1, tx_white,6, 100);
	}
	else if (!strcmp(colour, "pink"))
	{
		HAL_UART_Transmit(&huart1, tx_pink,5, 100);
	}
	else if (!strcmp(colour, "grey"))
	{
		HAL_UART_Transmit(&huart1, tx_grey,5, 100);
	}
	else if (!strcmp(colour, "brown"))
	{
		HAL_UART_Transmit(&huart1, tx_brown,6, 100);
	}
	HAL_Delay(4000);
}


char* hsv_string(struct HSV hsv_c){
    struct HSV all_colors_hsv[] = {
    {0, 1.00, 1.00, "red"},
    {120, 1.00, 1.00, "green"},
    {240, 1.00, 1.00, "blue"},
    {60, 1.00, 1.00, "yellow"},
    {0, 0.00, 0.00, "black"},
    {0, 0.00, 1.00, "white"},
    {0, 0.00, 0.50, "grey"},
    {39, 1.00, 1.00, "orange"},
    {350, 0.25, 1.00, "pink"},
    {0, 0.75, 0.65, "brown"},
    {300, 1.00, 0.50, "purple"},
    };

    double min_dis = 10000;
    int all_c_index = 0;

    double distance;
    struct HSV select_color;

    for (int i = 0; i < (11); i++){
        select_color = all_colors_hsv[i];
        distance = sqrt(pow(hsv_c.Hue - select_color.Hue, 2) + pow(hsv_c.Saturation - select_color.Saturation, 2) + pow(hsv_c.Value - select_color.Value, 2));

        if (distance < min_dis){
            all_c_index = i;
            min_dis = distance;

        }
    }

    if (all_c_index == 0){
        return "red";
    } else if (all_c_index == 3){
        return "green";
    }else if (all_c_index == 2){
        return "blue";
    }else if (all_c_index == 7){
        return "yellow";
    }else if (all_c_index == 4){
        return "black";
    }else if (all_c_index == 5){
        return "white";
    }else if (all_c_index == 6){
        return "grey";
    }else if (all_c_index == 7){
        return "orange";
    }else if (all_c_index == 8){
        return "pink";
    }else if (all_c_index == 9){
        return "brown";
    }else if (all_c_index == 10){
        return "purple";
    }

}
float hue(int a, int b, int c) {
	float hsv; // Array to store hue, saturation, value
	float gNorm = b / 255.0f;
	float rNorm = a / 255.0f;
	float bNorm = c / 255.0f;

	float max = fmaxf(rNorm, fmaxf(gNorm, bNorm));
	float min = fminf(rNorm, fminf(gNorm, bNorm));
	float d = max - min;

	if (d == 0) {
	        hsv = 0;
	    } else if (max == rNorm) {
	        hsv = fmodf(((gNorm - bNorm) / d), 6.0f) * 60.0f;
	        if (hsv < 0) hsv += 360.0f;
	    } else if (max == gNorm) {
	        hsv = ((bNorm - rNorm) / d + 2.0f) * 60.0f;
	    } else {
	        hsv = ((rNorm - gNorm) / d + 4.0f) * 60.0f;
	    }
	return hsv;
}

float saturation(int a, int b, int c) {
	float hsv; // Array to store hue, saturation, value
	float gNorm = b / 255.0f;
	float rNorm = a / 255.0f;
	float bNorm = c / 255.0f;

	float max = fmaxf(rNorm, fmaxf(gNorm, bNorm));
	float min = fminf(rNorm, fminf(gNorm, bNorm));
	float d = max - min;

	hsv = (max == 0) ? 0 : (d / max) * 100.0;

	return hsv;
}

float value(int a, int b, int c) {
	float hsv; // Array to store hue, saturation, value
	float gNorm = b / 255.0f;
	float rNorm = a / 255.0f;
	float bNorm = c / 255.0f;

	float max = fmaxf(rNorm, fmaxf(gNorm, bNorm));
	float min = fminf(rNorm, fminf(gNorm, bNorm));
	float d = max - min;

	hsv = max * 100.0;

	return hsv;
}

void write8 (uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
void enable(void);
void disable(void);
void setIntegrationTime(uint8_t it);
void setGain(uint8_t gain);
void tcs3272_init( void );
void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
void getRGB(int *R, int *G, int *B);

void write8 (uint8_t reg, uint32_t value)
{
    uint8_t txBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    txBuffer[1] = (value & 0xFF);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 2, 100);
}

uint8_t read8(uint8_t reg)
{
    uint8_t buffer[1];
    buffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, buffer, 1, 100);
    return buffer[0];
}

uint16_t read16(uint8_t reg)
{
  uint16_t ret;
    uint8_t txBuffer[1],rxBuffer[2];
    txBuffer[0] = (TCS34725_COMMAND_BIT | reg);
    HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, txBuffer, 1, 100);
    HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, rxBuffer, 2, 100);
    ret = rxBuffer[1];
    ret <<= 8;
    ret |= rxBuffer[0] & 0xFF;
  return ret;
}

void enable(void)
{
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
  HAL_Delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  HAL_Delay(50);
}

void disable(void)
{
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

void setIntegrationTime(uint8_t itime)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_ATIME, itime);
}

void setGain(uint8_t gain)
{
  if (_tcs34725Initialised == 0) tcs3272_init();
  write8(TCS34725_CONTROL, gain);
}

void tcs3272_init(void)
{
  /* Make sure we're actually connected */
  uint8_t readValue = read8(TCS34725_ID);
  if ((readValue != 0x44) && (readValue != 0x10) && (readValue != 0x4d))
  {
    return;
  }
  _tcs34725Initialised = 1;
  setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  setGain(TCS34725_GAIN_4X);
  enable();
}

void getRawData (uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c)
{
  if (_tcs34725Initialised == 0) tcs3272_init();

  *c = read16(TCS34725_CDATAL);
  *r = read16(TCS34725_RDATAL);
  *g = read16(TCS34725_GDATAL);
  *b = read16(TCS34725_BDATAL);
  HAL_Delay(50);
}

/* Get Red, Green and Blue color from Raw Data */
void getRGB(int *R, int *G, int *B)
{
    uint16_t rawRed, rawGreen, rawBlue, rawClear;
    getRawData(&rawRed, &rawGreen, &rawBlue, &rawClear);
    if(rawClear == 0)
    {
      *R = 0;
      *G = 0;
      *B = 0;
    }
    else
    {
    	*R = ((int)rawRed * 255 / rawClear);
    	*G = ((int)rawGreen * 255 / rawClear);
    	*B = ((int)rawBlue * 255 / rawClear);
    }
}

//const char* rgbToHex(int red, int green, int blue) {
//    static char hex[8];
//    sprintf(hex, "#%02X%02X%02X", red, green, blue);
//    return hex;
//}
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
