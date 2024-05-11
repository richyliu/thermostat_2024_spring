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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  PID_TypeDef TPID;
  double input;
  double output;
  double sp; // set point
} PID_control;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// when the heater is on (pwm set to 0), the ADC reading is on average different by this much than what it should be
#define HEATER_ADC_DIFF 0
// raw temp value (from 0 to 4096) is converted to temperature (Fahrenheit) using the equation:
// Ax + B
// where x is the raw temp value and A and B are constants defined below
#define RAW_TO_TEMP_A 0.2
#define RAW_TO_TEMP_B -38.5

#define SP_HIGH 97.0
#define SP_LOW 93.0
#define SP_INIT 95.0

#define _UART_BUF_SIZE 256
#define HIST_SIZE 4

//#define MANUAL_MODE

// settings for debug graph
#define GRAPH_N_CHARS 64
#define GRAPH_TEMP_MIN 90
#define GRAPH_TEMP_RANGE 10.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_PRINTF(...) \
    do { \
        snprintf(_uart_buf, sizeof(_uart_buf), __VA_ARGS__); \
        HAL_UART_Transmit(&huart2, (uint8_t*)_uart_buf, strnlen((const char*)_uart_buf, sizeof(_uart_buf)), 100); \
    } while (0)

#define UART_PRINT_FLOAT(val) \
  do { \
    if ((val) < 0) { \
      UART_PRINTF("-"); \
      UART_PRINTF("%lu.%02lu", (uint32_t)(-(val)), (uint32_t)(((-(val)) - (uint32_t)(-(val)))*100.0)); \
    } else { \
      UART_PRINTF("%lu.%02lu", (uint32_t)(val), (uint32_t)(((val) - (uint32_t)(val))*100.0)); \
    } \
  } while (0)

/**
 * Convert raw ADC readings to temperature (Fahrenheit)
 * Corrects for lower readings when heater is on
 */
#define RAW_TO_TEMP(raw) ((RAW_TO_TEMP_A*((raw) - heater_set_val*HEATER_ADC_DIFF) + RAW_TO_TEMP_B))

#define CLAMP(v, min, max) ((v) < (min) ? (min) : (v) > (max) ? (max) : (v))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char _uart_buf[_UART_BUF_SIZE];

// value the heater was set to
float heater_set_val;

PID_control pidctl;

float temp_history[HIST_SIZE];
float output_history[HIST_SIZE];
int current_history = 0;

int button_on = 0;
uint8_t button_debounce = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint16_t read_temp_raw() {
  HAL_ADC_Start(&hadc);
  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
  return HAL_ADC_GetValue(&hadc);
}

/**
 * Set heater with 0.0 being off and 1.0 being on (full)
 */
void set_heater(float value) {
  /*
   * Note:
   * raw value = 0: heater off 0%
   * raw value = 65535 (max): heater on 100%
   *
   * New version of hardware model is active high
   */
  heater_set_val = value;
  TIM1->CCR1 = fabs(value) * 0xffff;
}

/**
 * Sum multiple samples of ADC data spaced evenly by a delay
 */
float read_samples(int size, uint32_t tick_delay) {
  uint32_t total = 0;
  for (int i = 0; i < size; i++) {
    total += read_temp_raw();
    HAL_Delay(tick_delay);
  }
  return total * 1.0 / size;
}

/**
 * For calibrating raw temperature readings to real temperature.
 */
void calibrate_temperature() {
  // disable heater
  set_heater(0);
  UART_PRINTF("%d\n", sizeof(int));

  while (1) {
    float reading = read_samples(256, 10);

    UART_PRINTF("Raw reading averaged over 256 samples every 10 ms: ");
    UART_PRINT_FLOAT(reading);
    UART_PRINTF(", converted temp (F): ");
    UART_PRINT_FLOAT(RAW_TO_TEMP(reading));
    UART_PRINTF("\n\r");

    HAL_Delay(500);
  }
}

/**
 * For calibrating how much the ADC readings are affected by different heater settings.
 *
 * Nice way to analyze these values on command line (for average and stddev):
 * cut -d' ' -f 6,9 | sort | awk -F' ' '{vals[$1]+=$2;vsq[$1]+=($2)^2;c[$1]+=1;} END{for (v in vals) print v, vals[v]/c[v], sqrt((vsq[v]-vals[v]^2/c[v])/c[v]);}'
 */
void calibrate_voltage_drop() {
  float heatval = 0;
  float heater_init = 0.9;

  // disable heater
  set_heater(heater_init);

  while (1) {
    // calculate baseline
    set_heater(heater_init);
    float baseline = read_samples(256, 10);

    // measure spike with heater
    set_heater(heatval);
    float with_heater = read_samples(16, 2);
    set_heater(heater_init);

    UART_PRINTF("baseline: ");
    UART_PRINT_FLOAT(baseline);
    UART_PRINTF(", with heater initially at ");
    UART_PRINT_FLOAT(heater_init);
    UART_PRINTF(" with change to ");
    UART_PRINT_FLOAT(heatval);
    UART_PRINTF(": ");
    UART_PRINT_FLOAT(with_heater);
    UART_PRINTF(", delta: ");
    UART_PRINT_FLOAT(baseline - with_heater);
    UART_PRINTF("\n\r");

    HAL_Delay(500);
  }
}

void PID_init() {
  memset(&pidctl, 0, sizeof(pidctl));
  PID(&pidctl.TPID, &pidctl.input, &pidctl.output, &pidctl.sp, 2.0, 1000.0, 0, _PID_P_ON_M, _PID_CD_DIRECT);
  PID_SetMode(&pidctl.TPID, _PID_MODE_AUTOMATIC);
  PID_SetSampleTime(&pidctl.TPID, 80);
  PID_SetOutputLimits(&pidctl.TPID, 0, 1);
}

void PID_update() {
  pidctl.input = RAW_TO_TEMP(read_temp_raw());
  PID_Compute(&pidctl.TPID);
#ifdef MANUAL_MODE
  set_heater(pidctl.sp < SP_INIT);
#else
  set_heater(pidctl.output);
#endif

  // update history for debugging
  temp_history[current_history] = pidctl.input;
  output_history[current_history] = pidctl.output;
  current_history = (current_history + 1) % HIST_SIZE;
}

void PID_new_sp(double new_sp) {
  pidctl.sp = new_sp;
}

void button_released() {
  if (pidctl.sp < SP_INIT)
    PID_new_sp(SP_HIGH);
  else
    PID_new_sp(SP_LOW);
}

/**
 * Creates new row of temperature (and control) graph
 */
void graph_temp() {
  int tick = HAL_GetTick();
  int seconds = tick/1000;
  int ms = tick % 1000;

  float avg_i = 0, avg_o = 0;
  for (int i = 0; i < HIST_SIZE; i++) {
    avg_i += temp_history[i];
    avg_o += output_history[i];
  }
  avg_i /= HIST_SIZE;
  avg_o /= HIST_SIZE;

  int temp_col = (avg_i-GRAPH_TEMP_MIN)/GRAPH_TEMP_RANGE * GRAPH_N_CHARS;
  int out_col = avg_o * GRAPH_N_CHARS;
  int sp_col = (pidctl.sp-GRAPH_TEMP_MIN)/GRAPH_TEMP_RANGE * GRAPH_N_CHARS;
  temp_col = CLAMP(temp_col, 0, GRAPH_N_CHARS-1);
  out_col = CLAMP(out_col, 0, GRAPH_N_CHARS-1);
  sp_col = CLAMP(sp_col, 0, GRAPH_N_CHARS-1);

  char data[GRAPH_N_CHARS+2];
  memset(data, ' ', sizeof(data));
  data[0] = '|';
  data[GRAPH_N_CHARS-1] = '|';
  data[sp_col] = '.';
  data[out_col] = '+';
  data[temp_col] = '#';
  data[GRAPH_N_CHARS] = 0;
  data[GRAPH_N_CHARS+1] = 0;
  UART_PRINTF(data);

  UART_PRINTF(" temp: ");
  UART_PRINT_FLOAT(avg_i);
  UART_PRINTF(", control: ");
  UART_PRINT_FLOAT(avg_o);
  UART_PRINTF(", sp: ");
  UART_PRINT_FLOAT(pidctl.sp);
  UART_PRINTF(", raw: %4d, time: %3d.%03d sec\n", read_temp_raw(), seconds, ms);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  static int ctr = 0;

  if (htim->Instance == TIM16) {
    // interrupts every 10ms
    PID_update();

    // button debouncing
    int pressed = HAL_GPIO_ReadPin(blue_btn_GPIO_Port, blue_btn_Pin) == GPIO_PIN_RESET;
    button_debounce = (button_debounce << 1) | (pressed & 1);
    if (button_debounce == 0xff) {
      button_on = 1;
    } else if (button_debounce == 0x00) {
      if (button_on) {
        // button release (falling edge)
        button_released();
      }
      button_on = 0;
    }

    ctr++;
    // debug graph
    if (ctr % HIST_SIZE == 0) {
      graph_temp();
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  set_heater(0);

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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  // start PWM for heater control
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  // PID control
  PID_init();

  // reset heater again just to be sure
  set_heater(0);

  UART_PRINTF("Boot.\n\r");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* only run one of the below functions */

//  calibrate_temperature();
//  calibrate_voltage_drop();
  HAL_TIM_Base_Start_IT(&htim16); // start pid timer interrups

  PID_new_sp(SP_INIT);

  while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 51-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : blue_btn_Pin */
  GPIO_InitStruct.Pin = blue_btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(blue_btn_GPIO_Port, &GPIO_InitStruct);

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
  while (1) {
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
