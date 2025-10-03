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


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_SAMPLES 1024
#define ARM_MATH_CM7
#define fft_buf_size 1024
#define REF_RESISTANCE 2400.0f    // Ohms (Ri = 3.66kΩ)
#define ADC_VREF 3.3f             // ADC reference voltage
#define ADC_MAX_VALUE 4095.0f     // For a 12-bit ADC (2^12 - 1)

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float value = 0.2;

uint32_t var;

float32_t fft_input_v1[NUM_SAMPLES * 2];
float32_t fft_input_v2[NUM_SAMPLES * 2];
float32_t fft_output_v1[NUM_SAMPLES];  // Magnitude spectrum
float32_t fft_output_v2[NUM_SAMPLES];  // Magnitude spectrum
// Results
typedef struct {
    float magnitude_v1;    // V1 magnitude at signal frequency
    float magnitude_v2;    // V2 magnitude at signal frequency
    float phase_v1;        // V1 phase (radians)
    float phase_v2;        // V2 phase (radians)
    float phase_diff;      // V2 - V1 phase difference
    float impedance_mag;   // |Z(ω)|
    float impedance_phase; // ∠Z(ω)
    float real_z;          // Real part of Z
    float imag_z;          // Imaginary part of Z
    float I_component;     // In-phase component
    float Q_component;     // Quadrature component
} ImpedanceResult;

ImpedanceResult result;

// FFT instance (use appropriate size)
arm_rfft_fast_instance_f32 fft_instance;
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
	sine_LUT[i] = ((arm_cos_f32(theta)*0.95)+1) * (1 << dac_res) /2; //adding headroom by scaling down from 100% to 95% to remove clipping
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
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim7);
  //uint16_t dac_val = 0;
  HAL_Delay(1000);
  arm_rfft_fast_init_f32(&fft_instance,NUM_SAMPLES);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t* )adc1_buf, NUM_SAMPLES);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t* )adc3_buf, NUM_SAMPLES);
  HAL_TIM_Base_Start(&htim6);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
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
  htim6.Init.Period = 120-1;
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
  htim7.Init.Period = 1200-1;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|BLUELED_Pin, GPIO_PIN_RESET);

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

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void Prepare_FFT_Data(uint16_t* adc_buf_signed, uint16_t* adc_buf_unsigned,
                      float32_t* fft_in_v1, float32_t* fft_in_v2) {
    float32_t sum_v1 = 0.0f;
    float32_t sum_v2 = 0.0f;

    // Calculate DC offset (mean value)
    for (int i = 0; i < NUM_SAMPLES; i++) {
        sum_v1 += (float32_t)adc_buf_signed[i];
        sum_v2 += (float32_t)adc_buf_unsigned[i];
    }

    float32_t mean_v1 = sum_v1 / NUM_SAMPLES;
    float32_t mean_v2 = sum_v2 / NUM_SAMPLES;

    // Prepare FFT input: remove DC, apply window, format for CMSIS-DSP
    // Format: [real0, imag0, real1, imag1, ...] where all imag = 0 initially
    for (int i = 0; i < NUM_SAMPLES; i++) {
        // Remove DC offset
        float32_t sample_v1 = (float32_t)adc_buf_signed[i] - mean_v1;
        float32_t sample_v2 = (float32_t)adc_buf_unsigned[i] - mean_v2;

        // Optional: Apply Hanning window to reduce spectral leakage
        float32_t window = 0.5f * (1.0f - arm_cos_f32(2.0f * PI * i / NUM_SAMPLES));

        // Store in FFT input buffer (real part only)
        fft_in_v1[i] = sample_v1 * window;
        fft_in_v2[i] = sample_v2 * window;
    }
}

void Compute_FFT(float32_t* fft_input, float32_t* fft_output) {
    // Temporary buffer for FFT output (complex format)
    float32_t fft_complex[NUM_SAMPLES * 2];

    // Perform FFT
    arm_rfft_fast_f32(&fft_instance, fft_input, fft_complex, 0);

    // Calculate magnitude spectrum
    arm_cmplx_mag_f32(fft_complex, fft_output, NUM_SAMPLES / 2);


}

void Extract_Signal_Parameters(float32_t* fft_complex, uint32_t bin_index,
        float32_t* magnitude, float32_t* phase) {
// Get real and imaginary parts at the signal frequency bin
float32_t real = fft_complex[bin_index * 2];
float32_t imag = fft_complex[bin_index * 2 + 1];

float32_t magnitude_lsb = 2.0f * sqrtf(real * real + imag * imag) / NUM_SAMPLES;
   *magnitude = magnitude_lsb * (ADC_VREF / ADC_MAX_VALUE);

// Calculate magnitude (divide by NUM_SAMPLES/2 for proper scaling)
//*magnitude = 2.0f * sqrtf(real * real + imag * imag) / NUM_SAMPLES;

// Calculate phase
*phase = atan2f(imag, real);
}

void Calculate_IQ_Components(float32_t mag_v2, float32_t phase_diff,
                             float32_t* I, float32_t* Q) {
    // I = magnitude * cos(phase)
    *I = mag_v2 * arm_cos_f32(phase_diff);

    // Q = magnitude * sin(phase)
    *Q = mag_v2 * arm_sin_f32(phase_diff);
}

void Calculate_Impedance(ImpedanceResult* res) {
    // Voltage ratio V2/V1
    float32_t voltage_ratio = res->magnitude_v2 / res->magnitude_v1;

    // Phase difference (V2 - V1)
    res->phase_diff = res->phase_v2 - res->phase_v1;

    // Normalize phase to [-π, π]
    while (res->phase_diff > PI) res->phase_diff -= 2.0f * PI;
    while (res->phase_diff < -PI) res->phase_diff += 2.0f * PI;

    // Calculate I and Q
    Calculate_IQ_Components(res->magnitude_v2, res->phase_diff,
                           &res->I_component, &res->Q_component);

    // Z(ω) = -Ri × (V2/V1)
    // Magnitude
    res->impedance_mag = REF_RESISTANCE * voltage_ratio;

    // Phase (add 180° for the negative sign)
    res->impedance_phase = res->phase_diff + PI;

    // Normalize phase
    while (res->impedance_phase > PI) res->impedance_phase -= 2.0f * PI;
    while (res->impedance_phase < -PI) res->impedance_phase += 2.0f * PI;

    // Convert to rectangular form
    res->real_z = res->impedance_mag * arm_cos_f32(res->impedance_phase);
    res->imag_z = res->impedance_mag * arm_sin_f32(res->impedance_phase);
}

void Process_Impedance_Measurement(void) {
    // Wait for DMA completion (implement your own flags/callbacks)
    // while (!adc1_complete || !adc3_complete);
	char buffer[32];

    // Calculate frequency bin for signal
    uint32_t signal_bin = (uint32_t)((freq_out * NUM_SAMPLES) / fsampling);
    printf("\r\n Starting Impedance Measurement \r\n");
    printf("Signal frequency: %lu Hz\r\n", freq_out);
    printf("FFT bin: %lu\r\n", signal_bin);

    // STEP 3: Prepare data
    Prepare_FFT_Data(adc1_buf, adc3_buf, fft_input_v1, fft_input_v2);

    // STEP 4: Compute FFT for both channels
    float32_t fft_complex_v1[NUM_SAMPLES * 2];
    float32_t fft_complex_v2[NUM_SAMPLES * 2];

    arm_rfft_fast_f32(&fft_instance, fft_input_v1, fft_complex_v1, 0);
    arm_rfft_fast_f32(&fft_instance, fft_input_v2, fft_complex_v2, 0);

    // STEP 5: Extract signal parameters at the frequency of interest
    Extract_Signal_Parameters(fft_complex_v1, signal_bin,
                             &result.magnitude_v1, &result.phase_v1);
    Extract_Signal_Parameters(fft_complex_v2, signal_bin,
                             &result.magnitude_v2, &result.phase_v2);

    // STEP 7: Calculate impedance
    Calculate_Impedance(&result);
    // Results are now in 'result' structure
        // Print or transmit results
        printf("V1: %.3f V @ %.2f°\r\n", result.magnitude_v1,
               result.phase_v1 * 180.0f / PI);
        printf("V2: %.3f V @ %.2f°\r\n", result.magnitude_v2,
               result.phase_v2 * 180.0f / PI);
        printf("I: %.3f V, Q: %.3f V\r\n", result.I_component, result.Q_component);
        printf("|Z|: %.2f Ω, ∠Z: %.2f°\r\n", result.impedance_mag,
               result.impedance_phase * 180.0f / PI);
        printf("Z = %.2f %+.2fj Ω\r\n", result.real_z, result.imag_z);

    // Results are now in 'result' structure

        // Show frequency
        sprintf(buffer, "F = %lu Hz", freq_out);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // FFT bin
        sprintf(buffer, "Bin = %lu", signal_bin);
        ssd1306_SetCursor(0, 12);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // Voltages
        sprintf(buffer, "V1=%.2f", result.magnitude_v1);
        ssd1306_SetCursor(0, 24);
        ssd1306_WriteString(buffer, Font_7x10, White);

        sprintf(buffer, "V2=%.2f", result.magnitude_v2);
        ssd1306_SetCursor(64, 24);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // Impedance
        sprintf(buffer, "|Z|=%.1f", result.impedance_mag);
        ssd1306_SetCursor(0, 36);
        ssd1306_WriteString(buffer, Font_7x10, White);

        sprintf(buffer, "Ang=%.1f", result.impedance_phase * 180.0f / PI);
        ssd1306_SetCursor(64, 36);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // Complex form
        sprintf(buffer, "Z=%.1f%+.1fj", result.real_z, result.imag_z);
        ssd1306_SetCursor(0, 48);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // Refresh OLED
        ssd1306_UpdateScreen();
}

/*******************************************************************************
 * STEP 9: DMA CALLBACKS (OPTIONAL)
 ******************************************************************************/

volatile uint8_t adc1_complete = 0;
volatile uint8_t adc3_complete = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        adc1_complete = 1;
    } else if (hadc->Instance == ADC3) {
        adc3_complete = 1;
    }

    // If both complete, trigger processing
    if (adc1_complete && adc3_complete) {
        adc1_complete = 0;
        adc3_complete = 0;
        Process_Impedance_Measurement();
    }
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
