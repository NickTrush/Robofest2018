/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f7xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static unsigned int Parser (unsigned char a, unsigned char mask, unsigned char shift); // It's function for a short record of complex shifts
static int16_t Min_Max(int16_t variable, int16_t min, int16_t max); // Break off the ends
static void Engine_PWM_Set (int16_t Left_Power, int16_t Right_Power); // Set PWM setting on engines
static void Calibration_Channels(uint16_t * CH, double min_val, double max_val); // Aligns the sights on channels
static int16_t Switch_Range (int16_t prev_min_val, int16_t prev_max_val, int16_t min_val, int16_t max_val, int16_t value);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned char Radio_Data[250]; // Buffer for Radio module data
uint16_t CH[16]; // Processed channels
unsigned char Start_byte; // Byte only for start receiving
unsigned char Radio_flag = 0; // Access Radio flag
unsigned char DCH1; // DCH1 from sbus
unsigned char DCH2; // DCH2from sbus
unsigned char Frame_lost; // Frame lost from sbus
unsigned char failsafe; // failsafe from sbus
unsigned char tank_switch; // Flag set switching motion mod is ready

int16_t Left_Power = 0;
int16_t Right_Power = 0;
int16_t PWM_signal;

unsigned int i; // Counter
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  HAL_TIM_Base_Start_IT(&htim3);
  
  // PWM Activation
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  	//HAL_UART_Transmit(&huart2, (unsigned char *) "After While\r", 12, 0xff);

//  	HAL_UART_Transmit(&huart3, (unsigned char *) "While\r", 6, 0xff);
    if (Radio_flag == 1)
    {

      CH[0] = Radio_Data[1] + Parser(Radio_Data[2], 0b00000111, 8); // Right Left-Right
      CH[1] = Parser(Radio_Data[2], 0b11111000, -3) + Parser(Radio_Data[3], 0b00111111, 5); // Right Up-Down
      CH[2] = Parser(Radio_Data[3], 0b11000000, -6) + Parser(Radio_Data[4], 0b11111111, 2) + Parser(Radio_Data[5], 0b00000001, 10); // Left Up-Down
      CH[3] = Parser(Radio_Data[5], 0b11111110, -1) + Parser(Radio_Data[6], 0b00001111, 7); // Left Left-Right
      CH[4] = Parser(Radio_Data[6], 0b11110000, -4) + Parser(Radio_Data[7], 0b01111111, 4); // Far Left Swith
      CH[5] = Parser(Radio_Data[7], 0b10000000, -7) + Parser(Radio_Data[8], 0b11111111, 1) + Parser(Radio_Data[9], 0b00000011, 9); // Close Left-Left switch
      CH[6] = Parser(Radio_Data[9], 0b11111100, -2) + Parser(Radio_Data[10], 0b00011111, 6); // Close Left-Right switch
      CH[7] = Parser(Radio_Data[10], 0b11100000, -5) + Parser(Radio_Data[11], 0b11111111, 3); // Close Right-Left switch
      CH[8] = Radio_Data[12] + Parser(Radio_Data[13], 0b00000111, 8); // Close Right-Right switch
      CH[9] = Parser(Radio_Data[13], 0b11111000, -3) + Parser(Radio_Data[14], 0b00111111, 5); // Far Right Swith
      CH[10] = Parser(Radio_Data[14], 0b11000000, -6) + Parser(Radio_Data[15], 0b11111111, 2) + Parser(Radio_Data[16], 0b00000001, 10); // Left wheel
      CH[11] = Parser(Radio_Data[16], 0b11111110, -1) + Parser(Radio_Data[17], 0b00001111, 7); // Right wheel
      CH[12] = Parser(Radio_Data[17], 0b11110000, -4) + Parser(Radio_Data[18], 0b01111111, 4);
      CH[13] = Parser(Radio_Data[18], 0b10000000, -7) + Parser(Radio_Data[19], 0b11111111, 1) + Parser(Radio_Data[20], 0b00000011, 9);
      CH[14] = Parser(Radio_Data[20], 0b11111100, -2) + Parser(Radio_Data[21], 0b00011111, 6);
      CH[15] = Parser(Radio_Data[21], 0b11100000, -5) + Parser(Radio_Data[22], 0b11111111, 3);
      // Channels accepted

      DCH1 = Parser(Radio_Data[23], 0b00000001, 0);
      DCH2 = Parser(Radio_Data[23], 0b00000010, -1);
      Frame_lost = Parser(Radio_Data[23], 0b00000100, -2);
      failsafe = Parser(Radio_Data[23], 0b00001000, -3);
      // Specail signals accepted

      // Calibration
      Calibration_Channels(&CH[0], 172, 1811);
      Calibration_Channels(&CH[1], 160, 1792);
      Calibration_Channels(&CH[2], 172, 1808);
      Calibration_Channels(&CH[3], 128, 1792);

      if (CH[5] < 1024) // Wheels mod
      {
      	// Stop manipulator
      	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, Switch_Range(0, 2048, 544, 2400, 0));

      	// Set tank_switch
      	if (CH[2] == 1026)
      		tank_switch = 1;
		
      	// Here move settings
      	if ((CH[4] > 1024) && tank_switch)
      	{
      		Left_Power = (int16_t) ((CH[2] - 1024)*2);
      		Right_Power = (int16_t) ((CH[1] - 1024)*2);
      	}
      	else
      	{
      		tank_switch = 0;
      		Left_Power = (int16_t)((CH[1] + CH[0] - 2048)*(CH[2]/1024.0));
      		Right_Power = (int16_t)((CH[1] - CH[0])*(CH[2]/1024.0));
      	}
      		
      	// Normalization
      	Left_Power = Min_Max(Left_Power, -2048, 2048);
      	Right_Power = Min_Max(Right_Power, -2048, 2048);
      	
      	if (Frame_lost) // If the packege has been lost, we all turn off
      	{
      	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
      	}
      	else // Else set PWM setting on engines
      		Engine_PWM_Set(Left_Power, Right_Power);
      }
      else // Manipulator mod
      {
      	// Stop motion
      	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

      	PWM_signal = Switch_Range(0, 2048, 544, 2400, CH[2]);
      	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_signal);
      }

      Radio_flag = 0; //Receiving access
    }


  	//HAL_Delay(500);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3;
  PeriphClkInitStruct.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 80-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2048;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 12;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
  huart2.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF3 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
unsigned int Parser (unsigned char a, unsigned char mask, unsigned char shift)
{
  if (shift > 0)
    return (((unsigned int) (a&mask)) << shift);
  else
  	return (((unsigned int) (a&mask)) >> (-1 * shift));
}

int16_t Min_Max(int16_t variable, int16_t min, int16_t max)
{
	if (variable < min)
		return min;
	if (variable > max)
		return max;
	return variable;
}

void Engine_PWM_Set (int16_t Left_Power, int16_t Right_Power)
{
	/// LEFT
    if (Left_Power > 0) // Then the direct polarity
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
      // PWM settings
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Left_Power);
    }
    if (Left_Power < 0) // Then the invert polarity
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
      // PWM settings
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, -1*Left_Power);
    }
    /// RIGHT
    if (Right_Power > 0) // Then the direct polarity
    {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET);
      // PWM settings
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Right_Power);
    }
    if (Right_Power < 0) // Then the invert polarity
    {
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET);
      // PWM settings
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, -1*Right_Power);
    }
}

void Calibration_Channels(uint16_t * CH, double min_val, double max_val)
{
	*CH = (uint16_t) (2048.0*(*CH - min_val)/((double)(max_val - min_val)));
	*CH = (uint16_t) (Min_Max((int16_t)*CH, 0, 2048));
}

int16_t Switch_Range (int16_t prev_min_val, int16_t prev_max_val, int16_t min_val, int16_t max_val, int16_t value)
{
	return (int16_t) ((value - prev_min_val)*((max_val - min_val)/((double)(prev_max_val - prev_min_val))) + min_val);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
