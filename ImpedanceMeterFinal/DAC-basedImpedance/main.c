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
#include <string.h>
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"


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
#define ADC_VREF 3.3f
#define ADC_MAX_VALUE 4095.0f
#define UART_TIMEOUT 5000
#define START_MARKER 0xAA
#define END_MARKER 0x55

// Circuit models
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

DAC_HandleTypeDef hdac;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint16_t adc1_buf[NUM_SAMPLES];
uint16_t adc3_buf[NUM_SAMPLES];
uint16_t sine_LUT[NUM_SAMPLES];
uint32_t N = NUM_SAMPLES;
uint32_t M = 32;
uint32_t freq_out ;
uint32_t fsampling ; // Hz (adjust based on TIM6 config)
uint32_t phase_inc;
uint32_t phase_acc;
uint32_t dac_res = 12;
uint32_t lut_idx;
uint32_t k;


#define NUM_CALIBRATION_SAMPLES 30
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
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM7_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
/* USER CODE BEGIN PFP */
void MCP41100_SetValue(uint8_t value);
void Send_Data_To_Julia(void);
uint8_t Receive_Results_From_Julia(ProcessedResult* result);
void Update_Display(ProcessedResult* result);
void Calculate_Circuit_Parameters(ProcessedResult* result);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float32_t fft_input_v1[NUM_SAMPLES * 2];
float32_t fft_input_v2[NUM_SAMPLES * 2];
float32_t fft_output_v1[NUM_SAMPLES];  // Magnitude spectrum
float32_t fft_output_v2[NUM_SAMPLES];  // Magnitude spectrum
// Results

//ImpedanceResult result;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	for (int i = 0; i < NUM_SAMPLES; i++){
	float32_t theta = 2*PI * (float32_t)(i)/(float32_t)(NUM_SAMPLES);

	/// Converting sine value (-1 to +1) to DAC range from 0 to 4095 for 12-bit)
	sine_LUT[i] = ((arm_cos_f32(theta)*0.5)+1) * (1 << dac_res) /2; //adding headroom by scaling down from 100% to 95% to remove clipping
		}
		freq_out = 10;
		fsampling = 10000;

		//This represents the fraction of a complete sine cycle per sample
		phase_inc = ((float64_t)freq_out/ fsampling) * pow(2.0, M);
		phase_acc = 0;
		k = log2(NUM_SAMPLES);
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
  MX_DAC_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
  //uint16_t dac_val = 0;
  HAL_Delay(1000);
  //arm_rfft_fast_init_f32(&fft_instance,NUM_SAMPLES);
  MCP41100_SetValue(255);  ///(256-x/256)*100k

  HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc1_buf, NUM_SAMPLES);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t* )adc3_buf, NUM_SAMPLES);
  HAL_TIM_Base_Start(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  HAL_GPIO_TogglePin(BLUELED_GPIO_Port, BLUELED_Pin);  // Should blink
	  HAL_Delay(1500);
      if (result_ready) {
          result_ready = 0;
          Update_Display(&latest_result);

          // Small delay before next acquisition


          // Start next acquisition
          adc1_complete = 0;
          adc3_complete = 0;
          HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buf, NUM_SAMPLES);
          HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_buf, NUM_SAMPLES);
          HAL_Delay(100);
      }

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  hi2c1.Init.Timing = 0x2010091A;
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  htim6.Init.Prescaler = 8-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1200-1;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 119;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|BLUELED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : POT_CS_Pin */
  GPIO_InitStruct.Pin = POT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POT_CS_GPIO_Port, &GPIO_InitStruct);

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

//if (htim == &htim7){
//		 //HAL_GPIO_TogglePin(Green_Led_GPIO_Port, Green_Led_Pin);
//	}
	lut_idx = phase_acc >> (M - k);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, sine_LUT[lut_idx]);
	phase_acc += phase_inc;

}
void MCP41100_SetValue(uint8_t value)
{
    uint8_t spiData[2];

    // Command byte 0x11 = Write to pot 0
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


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc1_complete = 1;
    } else if (hadc->Instance == ADC3) {
        adc3_complete = 1;
    }

    // If both complete, trigger processing
    if (adc1_complete && adc3_complete && !uart_busy) {
            adc1_complete = 0;
            adc3_complete = 0;
            uart_busy = 1;

            // Send adc1+3 data to Julia
            Send_Data_To_Julia();

            // Receive processed impedance results
            if (Receive_Results_From_Julia(&latest_result)) {
                Calculate_Circuit_Parameters(&latest_result);
                result_ready = 1;
            }

            uart_busy = 0;
        }
}

void Send_Data_To_Julia(void) {
    uint8_t header[9];

    // Send start marker
    header[0] = START_MARKER;

    // Send metadata
    memcpy(&header[1], &freq_out, 4);
    memcpy(&header[5], &fsampling, 4);

    HAL_UART_Transmit(&huart3, header, 9, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)adc1_buf, NUM_SAMPLES * 2, HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart3, (uint8_t*)adc3_buf, NUM_SAMPLES * 2, HAL_MAX_DELAY);

    uint8_t end = END_MARKER;
    HAL_UART_Transmit(&huart3, &end, 1, HAL_MAX_DELAY);
}

uint8_t Receive_Results_From_Julia(ProcessedResult* result) {
    uint8_t start_marker;
    uint8_t end_marker;

    // Wait for start marker with timeout
    if (HAL_UART_Receive(&huart3, &start_marker, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }

    if (start_marker != START_MARKER) {
        return 0;
    }

    // Receive the basic result structure (7 floats = 28 bytes)
    uint8_t model_id;
    uint8_t result_buffer[28];
    if (HAL_UART_Receive(&huart3, &model_id, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }
    result->circuit_model = model_id;

    if (HAL_UART_Receive(&huart3, result_buffer, 28, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }

    // Parse the results
    memcpy(&result->impedance_mag, &result_buffer[0], 4);
    memcpy(&result->impedance_phase, &result_buffer[4], 4);
    memcpy(&result->real_z, &result_buffer[8], 4);
    memcpy(&result->imag_z, &result_buffer[12], 4);
    memcpy(&result->magnitude_v1, &result_buffer[16], 4);
    memcpy(&result->magnitude_v2, &result_buffer[20], 4);
    memcpy(&result->std_err, &result_buffer[24], 4);

    // Wait for end marker
    if (HAL_UART_Receive(&huart3, &end_marker, 1, UART_TIMEOUT) != HAL_OK) {
        return 0;
    }

    if (end_marker != END_MARKER) {
        return 0;
    }

    return 1;
}

/* Circuit Parameter Calculation ---------------------------------------------*/
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
            float G = real_z / (real_z*real_z + imag_z*imag_z);
            float B = -imag_z / (real_z*real_z + imag_z*imag_z);
            result->param1 = 1.0f / G;  // Rp
            result->param2 = B / omega * 1e6f;  // Cp in µF
            break;
        }

        case MODEL_PARALLEL_RL: {
            float G = real_z / (real_z*real_z + imag_z*imag_z);
            float B = -imag_z / (real_z*real_z + imag_z*imag_z);
            result->param1 = 1.0f / G;  // Rp
            result->param2 = -1.0f / (omega * B) * 1e3f;  // Lp in mH
            break;
        }
    }
}


/* Display Functions-*/
void Update_Display(ProcessedResult* result) {
    char buffer[32];

    ssd1306_Fill(Black); //screen blank

    // Line 0 Frequency and Standard Error
    sprintf(buffer, "F=%luHz SE=%.2f", freq_out, result->std_err);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buffer, Font_6x8, White);

    // Line 1 Impedance Magnitude
    sprintf(buffer, "|Z|=%.1f", result->impedance_mag);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(buffer, Font_7x10, White);

    // Line 2 Phase Angle
    float phase_deg = result->impedance_phase * 180.0f / 3.14159265f;
    sprintf(buffer, "Ang=%.1f", phase_deg);
    ssd1306_SetCursor(64, 12);
    ssd1306_WriteString(buffer, Font_7x10, White);

    // Line 3 Complex Impedance
    sprintf(buffer, "Z=%.0f%+.0fj", result->real_z, result->imag_z);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString(buffer, Font_7x10, White);

    // Line 4-5 Circuit Parameters
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
