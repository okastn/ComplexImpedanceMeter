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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "math.h"
#include <stdio.h>
#include "arm_math.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float impedance_mag;
    float impedance_phase;
    float real_z;
    float imag_z;
    float magnitude_v1;
    float magnitude_v2;
    float std_err;
    uint8_t circuit_model;  // 1=Series RC, 2=Series RL, 3=Parallel RC, 4=Parallel RL
    float param1;  // Rs or Rp
    float param2;  // Cs, Ls, Cp, or Lp
} ProcessedResult;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES 1024
#define ARM_MATH_CM7
#define fft_buf_size 1024
#define REF_RESISTANCE 6250.250f//10406.25f    // Ohms 2.4/4.7/9.1 (Ri = 3.66kΩ)
#define ADC_VREF 3.3f             // ADC reference voltage
#define ADC_MAX_VALUE 4095.0f     // For a 12-bit ADC (2^12 - 1)
#define AD9833_MCLK 16000000UL  // 16 MHz crystal
#define START_MARKER 0xAA
#define END_MARKER 0x55
#define MODEL_SERIES_RC 1
#define MODEL_SERIES_RL 2
#define MODEL_PARALLEL_RC 3
#define MODEL_PARALLEL_RL 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t adc1_buf[NUM_SAMPLES];
uint16_t adc3_buf[NUM_SAMPLES];
uint16_t sine_LUT[NUM_SAMPLES];
uint32_t N = NUM_SAMPLES;
uint32_t M = 32;
uint32_t freq_out  = 10 ; //default - will be overwritten
uint32_t fsampling ; // Hz (adjust based on TIM6 config)
uint32_t phase_inc;
uint32_t phase_acc;
uint32_t dac_res = 12;
uint32_t lut_idx;
uint32_t k;

volatile uint8_t trigger = 0;
volatile uint8_t adc_running = 0;

#define NUM_CALIBRATION_SAMPLES 20
volatile uint8_t adc1_complete = 0;
volatile uint8_t adc3_complete = 0;
volatile uint8_t uart_busy = 0;

ProcessedResult latest_result;
uint8_t result_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
static void AD9833_SetFrequency(uint32_t freq_hz);
void MCP41100_SetValue(uint8_t value);
void Send_Data_To_Julia(void);
uint8_t Receive_Results_From_Julia(ProcessedResult* result);
void Update_Display(ProcessedResult* result);
void Calculate_Circuit_Parameters(ProcessedResult* result);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_Delay(1000);
  freq_out = 10;
  fsampling = 1000000;
  AD9833_SetFrequency(freq_out);

  MCP41100_SetValue(255); //R_A = 100k(256-tap)/256
  Test_UART_Direct();

  HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc1_buf, NUM_SAMPLES);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t* )adc3_buf, NUM_SAMPLES);
  HAL_TIM_Base_Start(&htim6);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //printf("ADC1: %u, ADC3: %u\r\n", adc1_buf[0], adc3_buf[0]);
	   //   HAL_Delay(500);
	  HAL_GPIO_TogglePin(BLUELED_GPIO_Port, BLUELED_Pin);
	  HAL_Delay(1500);

		if (result_ready) {
			result_ready = 0;
			Update_Display(&latest_result);

			/* Start next acquisition */
			adc1_complete = 0;
			adc3_complete = 0;
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, NUM_SAMPLES);
			HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, NUM_SAMPLES);
			HAL_Delay(100);	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T6_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.Timing = 0x0000030F;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|BLUELED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : POT_CS_Pin */
  GPIO_InitStruct.Pin = POT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(POT_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FSYNC_Pin */
  GPIO_InitStruct.Pin = FSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSYNC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin BLUELED_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|BLUELED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void AD9833_SetFrequency(uint32_t freq_hz)
{
    uint16_t data;
    uint32_t freq_word;

    // 28-bit frequency word calculation
    freq_word = (uint32_t)((freq_hz * (1ULL << 28)) / 16000000UL); // MCLK = 16 MHz

    // FSYNC Low to start
    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_RESET);

    /**
     * DB13 is set to 1.
     * This allows a complete word to be loaded into a
     * frequency register in two consecutive writes.
	 * The first write contains 14 LSBs.
	 * The second write contains 14 MSBs.
     * RESET bit DB8 is set to 1.
     * This resets internal registers to 0,
     * which corresponds to an analog output of midscale.
     ***/
    data = 0x2100;
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);

    /** Frequency word
     * DB15 and DB14 are set to 0 and 1, respectively,
     * which is the Frequency Register 0 address.
     **/
    // 2. Write lower 14 bits of frequency word
    data = 0x4000 | (freq_word & 0x3FFF);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);

    // 3. Write upper 14 bits of frequency word
    data = 0x4000 | ((freq_word >> 14) & 0x3FFF);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);

    // 4. Phase information
    /** DB15, DB14, and DB13 are set to 110, with DB12 set to don’t care (X),
     * respectively, which is the address for Phase Register 0.
     * The remaining 12 bits are the data bits and are all 0s in this case.
     * */
    //data = 0xC000;
    //HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);

    // 4. Phase information: 90° shift for cosine
        // Phase register = 0xC000 | (phase & 0x0FFF)
        // 12-bit phase word = (desired_phase * 4096) / (2π)
    uint16_t phase90 = 0xC000 | (0x0FFF & (uint16_t)(4096 * 0.25)); // 90° = 0.25 of full scale
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&phase90, 1, HAL_MAX_DELAY);

    // 5. Control register: Clear RESET to enable output
    data = 0x2000;  // RESET = 0, Mode = Sine
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);

    // FSYNC High to end
    HAL_GPIO_WritePin(FSYNC_GPIO_Port, FSYNC_Pin, GPIO_PIN_SET);
}

void Test_UART_Direct(void) {
	      char test_msg[] = "UART Test - Direct Transmission\r\n";
	      HAL_UART_Transmit(&huart3, (uint8_t*)test_msg, strlen(test_msg), HAL_MAX_DELAY);
	  }


void MCP41100_SetValue(uint8_t value)
{
    uint8_t spiData[2];

    // Command byte: 0x11 = Write to pot 0
    spiData[0] = 0x11;
    spiData[1] = value;  // Wiper value (0-255)

    // CS Low
    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_RESET);

    // Small delay for CS setup time (minimum 100ns, 1us is safe)
    HAL_Delay(1);

    // Transmit 2 bytes
    HAL_SPI_Transmit(&hspi4, spiData, 2, HAL_MAX_DELAY);

    // Small delay before CS high (minimum 100ns)
    HAL_Delay(1);

    // CS High
    HAL_GPIO_WritePin(POT_CS_GPIO_Port, POT_CS_Pin, GPIO_PIN_SET);
}



int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}


/* ADC conversion complete callback (both ADC1 & ADC3) */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc1_complete = 1;
    } else if (hadc->Instance == ADC3) {
        adc3_complete = 1;
    }

    /* If both complete, send to Julia for processing (unless UART already busy) */
    if (adc1_complete && adc3_complete && !uart_busy) {
        adc1_complete = 0;
        adc3_complete = 0;
        uart_busy = 1;

        /* Send adc1+adc3 data to Julia */
        Send_Data_To_Julia();

        /* Receive processed impedance results from Julia */
        if (Receive_Results_From_Julia(&latest_result)) {
            Calculate_Circuit_Parameters(&latest_result);
            result_ready = 1;


        }

        uart_busy = 0;
    }
}

/* Send binary packet to Julia:
   [START_MARKER][freq_out(4)][fsampling(4)][adc1_buf(2048B)][adc3_buf(2048B)][END_MARKER]
   Note: adc buffers are 1024 samples x 2 bytes = 2048 bytes each
*/
void Send_Data_To_Julia(void) {
    uint8_t header[9];
    header[0] = START_MARKER;

    memcpy(&header[1], &freq_out, 4);
    memcpy(&header[5], &fsampling, 4);

    HAL_UART_Transmit(&huart3, header, 9, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)adc1_buf, NUM_SAMPLES * 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)adc3_buf, NUM_SAMPLES * 2, HAL_MAX_DELAY);

    uint8_t end = END_MARKER;
    HAL_UART_Transmit(&huart3, &end, 1, HAL_MAX_DELAY);
}

/* Receive results back from Julia:
   [START_MARKER][model_id(1)][7 floats (28 bytes)][END_MARKER]
*/
uint8_t Receive_Results_From_Julia(ProcessedResult* result) {
    uint8_t start_marker;
    uint8_t end_marker;

    if (HAL_UART_Receive(&huart3, &start_marker, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }
    if (start_marker != START_MARKER) {
        return 0;
    }

    uint8_t model_id;
    uint8_t result_buffer[28];
    if (HAL_UART_Receive(&huart3, &model_id, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }
    result->circuit_model = model_id;

    if (HAL_UART_Receive(&huart3, result_buffer, 28, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }

    memcpy(&result->impedance_mag, &result_buffer[0], 4);
    memcpy(&result->impedance_phase, &result_buffer[4], 4);
    memcpy(&result->real_z, &result_buffer[8], 4);
    memcpy(&result->imag_z, &result_buffer[12], 4);
    memcpy(&result->magnitude_v1, &result_buffer[16], 4);
    memcpy(&result->magnitude_v2, &result_buffer[20], 4);
    memcpy(&result->std_err, &result_buffer[24], 4);

    if (HAL_UART_Receive(&huart3, &end_marker, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }
    if (end_marker != END_MARKER) {
        return 0;
    }
    return 1;
}

/* Circuit Parameter Calculation  */
void Calculate_Circuit_Parameters(ProcessedResult* result) {
    float omega = 2.0f * 3.14159265f * (float)freq_out;
    float real_z = result->real_z;
    float imag_z = result->imag_z;

    switch (result->circuit_model) {
        case MODEL_SERIES_RC:
            result->param1 = real_z;
            result->param2 = -1.0f / (omega * imag_z) * 1e6f;  // Cs in µF
            break;

        case MODEL_SERIES_RL:
            result->param1 = real_z;
            result->param2 = imag_z / omega * 1e3f;  // Ls in mH
            break;

        case MODEL_PARALLEL_RC: {
            float denom = (real_z*real_z + imag_z*imag_z);
            float G = real_z / denom;
            float B = -imag_z / denom;
            result->param1 = 1.0f / G;  // Rp
            result->param2 = B / omega * 1e6f;  // Cp in µF
            break;
        }

        case MODEL_PARALLEL_RL: {
            float denom = (real_z*real_z + imag_z*imag_z);
            float G = real_z / denom;
            float B = -imag_z / denom;
            result->param1 = 1.0f / G;  // Rp
            result->param2 = -1.0f / (omega * B) * 1e3f;  // Lp in mH
            break;
        }
    }
}

/* Update OLED with results */
void Update_Display(ProcessedResult* result) {
    char buffer[32];
    ssd1306_Fill(Black);

    sprintf(buffer, "F=%luHz SE=%.2f", freq_out, result->std_err);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_6x8, White);

    sprintf(buffer, "|Z|=%.1f", result->impedance_mag);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(buffer, Font_7x10, White);

    float phase_deg = result->impedance_phase * 180.0f / 3.14159265f;
    sprintf(buffer, "Ang=%.1f", phase_deg);
    ssd1306_SetCursor(64, 12);
    ssd1306_WriteString(buffer, Font_7x10, White);

    sprintf(buffer, "Z=%.0f%+.0fj", result->real_z, result->imag_z);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString(buffer, Font_7x10, White);

    ssd1306_SetCursor(0, 36);
    switch(result->circuit_model) {
        case MODEL_SERIES_RC:
            sprintf(buffer, "Rs=%.1f", result->param1);
            ssd1306_WriteString(buffer, Font_7x10, White);
            sprintf(buffer, "Cs=%.6fuF", result->param2);
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString(buffer, Font_7x10, White);
            break;

        case MODEL_SERIES_RL:
            sprintf(buffer, "Rs=%.1f", result->param1);
            ssd1306_WriteString(buffer, Font_7x10, White);
            sprintf(buffer, "Ls=%.2fmH", result->param2);
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString(buffer, Font_7x10, White);
            break;

        case MODEL_PARALLEL_RC:
            sprintf(buffer, "Rp=%.1f", result->param1);
            ssd1306_WriteString(buffer, Font_7x10, White);
            sprintf(buffer, "Cp=%.6fuF", result->param2);
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString(buffer, Font_7x10, White);
            break;

        case MODEL_PARALLEL_RL:
            sprintf(buffer, "Rp=%.1f", result->param1);
            ssd1306_WriteString(buffer, Font_7x10, White);
            sprintf(buffer, "Lp=%.2fmH", result->param2);
            ssd1306_SetCursor(0, 48);
            ssd1306_WriteString(buffer, Font_7x10, White);
            break;
    }

    ssd1306_UpdateScreen();
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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
#ifdef USE_FULL_ASSERT
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
