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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "aprs.h"


/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;			// Baud rate counter
TIM_HandleTypeDef htim6;			// Timer for DAC
TIM_HandleTypeDef htim7;			// Unused in demo code - used for WSPR
HAL_DAC_StateTypeDef rc;
uint32_t	rc2;

MyGlobals Globals;
Si5351 MySi5351;

char gps_time[7] = "220101";       // HHMMSS
char gps_aprs_lat[9] = "4226.99N";
char gps_aprs_lon[10] = "07628.88W";
float gps_course = 0.0;
float gps_speed = 0.0;
float gps_altitude = 1234.5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


#define XMITOFF(clock) 	    MySi5351.output_enable(clock, DISABLE);
#define XMITON(clock) 	    MySi5351.output_enable(clock, ENABLE);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_DAC_Init(void);
static void MX_DMA_Init(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

	Globals.I2cHandle.Instance = I2C1;
	Globals.I2cHandle.Init.ClockSpeed = 400000;
	Globals.I2cHandle.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
	Globals.I2cHandle.Init.OwnAddress1 = 0;
	Globals.I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	Globals.I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	Globals.I2cHandle.Init.OwnAddress2 = 0;
	Globals.I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	Globals.I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    if (HAL_I2C_Init(&Globals.I2cHandle) != HAL_OK) {
  	  Error_Handler(__FILE__, __LINE__);
    }
}

void MX_I2C1_DeInit()
{
	if (HAL_I2C_DeInit(&Globals.I2cHandle) != HAL_OK) {
		  Error_Handler(__FILE__, __LINE__);
	}
}

void
Si5351Init()
{
	MX_I2C1_Init();	// Initialize I2C bus

	// ASTXR-12-26.000MHz-512545 TCXO is 26 MHZ.  We use 0PF, since it's a TCXO and not a crystal.
	MySi5351.init(SI5351_CRYSTAL_LOAD_0PF, 26000000UL, 0UL);

    // power down the unused clocks
    MySi5351.set_clock_pwr(SI5351_CLK1, DISABLE);
    MySi5351.set_clock_pwr(SI5351_CLK2, DISABLE);

    // Turn off transmit on unused clocks
    XMITOFF(SI5351_CLK1);
    XMITOFF(SI5351_CLK2);

    MySi5351.set_clock_pwr(SI5351_CLK0, ENABLE);
    XMITOFF(SI5351_CLK0);
	MySi5351.drive_strength(SI5351_CLK0,SI5351_DRIVE_8MA);	// Max power

    // Set correction as specified in config.h.
}

void
Si5351DeInit()
{
	MX_I2C1_DeInit();	// Initialize I2C bus
}


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
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_DAC_Init();



// xxxx


/*
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) 10) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }

  // xxxx
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) 1023) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) 2047) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) 3065) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t) 4095) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
  {
	  Error_Handler(__FILE__, __LINE__);
  }
  rc = HAL_DAC_GetState(&hdac);
  rc2 = HAL_DAC_GetError(&hdac);
*/



  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Si5351Init();
#define NORMALx
#ifdef NORMAL
  // Normal FM
    MySi5351.set_freq(14433000000ULL, SI5351_CLK0);
    MySi5351.output_enable(SI5351_CLK0, ENABLE);
#else //Not Normal

  // FM + VCO
    MySi5351.output_enable(SI5351_CLK0, DISABLE);

  MySi5351.set_vcxo(86598000000ULL, 30);
  // Set CLK0 to be locked to VCXO
  MySi5351.set_ms_source(SI5351_CLK0, SI5351_PLLB);
  // Tune to 144.330 MHz center frequency
  MySi5351.set_freq_manual(14433000000ULL, 86598000000ULL, SI5351_CLK0);
  MySi5351.update_status();
#endif //NORMAL
  trace_puts("Hello,world!\n");



  while (1)
  {

	  aprs_send();

	  trace_printf("SYS_INIT: %d",MySi5351.dev_status.SYS_INIT );
	  trace_printf("  LOL_A: %d", MySi5351.dev_status.LOL_A);
	  trace_printf("  LOL_B: %d", MySi5351.dev_status.LOL_B);
	  trace_printf("  LOS: %d", MySi5351.dev_status.LOS);
	  trace_printf("  REVID: %d\n", MySi5351.dev_status.REVID);
	  trace_printf("\n");
	  HAL_Delay(5000);



  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  /**Initializes the CPU, AHB and APB busses clocks
  */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;	// 32mhz
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
		Error_Handler(__FILE__, __LINE__);
	  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
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

/* DAC init function */
static void MX_DAC_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization
    */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config
    */
//  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;

  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 13332;
  htim2.Init.Period = 1;			// 1200 hz
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  // 1200 hz
  htim6.Init.Prescaler = 100;
  htim6.Init.Period = 10;			// 1200 hz
  //htim6.Init.Period = 5;			// 2200 hz
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
  // We turn on the ARPE register so that the counter is only updated AFTER a clean
  // count completes.  Otherwise, we run the risk of overcounting an interval.
  htim6.Instance->CR1  |= TIM_CR1_ARPE; 	// Turn on auto-preload register

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
void MX_TIM6_DeInit(void)
{
  htim6.Instance = TIM6;
  if (HAL_TIM_Base_DeInit(&htim6) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 deinit function */
void MX_TIM2_DeInit(void)
{
  htim2.Instance = TIM2;
  if (HAL_TIM_Base_DeInit(&htim2) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 4999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 3;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  // xxxx
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : C0_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  /*Configure GPIO pin : C1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */



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
