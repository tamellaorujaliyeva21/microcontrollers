/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LD2_Pin        GPIO_PIN_5
#define LD2_GPIO_Port  GPIOA
#define B1_Pin         GPIO_PIN_13
#define B1_GPIO_Port   GPIOC
#define MODE_STOP                0
#define MODE_STANDBY             1
#define TEST_MODE                MODE_STANDBY   /* flip, rebuild, reflash */

#define STOP_USE_LPDS            1  /* PWR_CR.LPDS: 1 = low-power regulator, 0 = main regulator */
#define STOP_USE_FPDS            1  /* PWR_CR.FPDS: 1 = flash powered down in Stop */
#define RESTORE_CLOCK_AFTER_STOP 1  /* 0 = deliberately fall into the clock trap */

#define TRAP_LOOP_N              2000u

#define MARK_WAKE_HI()  (GPIOB->BSRR = (1u << 0))
#define MARK_WAKE_LO()  (GPIOB->BSRR = (1u << 16))
#define MARK_TRAP_HI()  (GPIOB->BSRR = (1u << 1))
#define MARK_TRAP_LO()  (GPIOB->BSRR = (1u << 17))

#define BKP_WAKE_COUNT  (RTC->BKP1R)   /* DR1; DR0 is left alone (CubeMX RTC code uses it) */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile uint32_t stop_wakes = 0;  /* lives in SRAM, lost in Standby */
static volatile uint8_t  in_stop    = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
static void led_blink(uint32_t on_ms, uint32_t off_ms, uint32_t n);
static void clock_trap_probe(void);
static void enter_stop(void);
static void enter_standby(void);
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
	  /* Wake marker, raised as early as possible after reset.
	     MX_GPIO_Init() later writes PB0 HIGH too, so it stays high until we sleep. */
	  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	  (void)RCC->AHB1ENR;
	  GPIOB->MODER   = (GPIOB->MODER & ~((3u << 0) | (3u << 2))) | (1u << 0) | (1u << 2);
	  GPIOB->OSPEEDR |= (3u << 0) | (3u << 2);
	  GPIOB->BSRR    = (1u << 0);
	  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  MX_GPIO_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();               /* DBP: needed for RTC->BKPxR after every reset */

    uint8_t standby_wake = (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET);
    if (standby_wake) {
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
      BKP_WAKE_COUNT++;                        /* survives Standby */
    } else {
      BKP_WAKE_COUNT = 0;                      /* cold boot / NRST: SBF = 0 */
    }

    clock_trap_probe();                        /* runs SystemClock_Config() between markers */

    if (standby_wake) {
      /* STANDBY WAKE: 3 very fast flashes, pause, then N slow blinks (N = count) */
      led_blink(40, 40, 3);
      HAL_Delay(400);
      led_blink(300, 300, BKP_WAKE_COUNT > 10 ? 10 : BKP_WAKE_COUNT);
    } else {
      /* COLD BOOT: one long 1 s ON */
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
      HAL_Delay(1000);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }

  #if TEST_MODE == MODE_STANDBY
    HAL_Delay(100);
    enter_standby();                           /* never returns */
  #endif
    /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  #if TEST_MODE == MODE_STOP
	      enter_stop();                            /* returns after EXTI wake, on HSI 16 MHz */
	    #if RESTORE_CLOCK_AFTER_STOP
	      clock_trap_probe();                      /* PLL is off after Stop: rebuild 84 MHz */
	    #endif
	      /* STOP WAKE: N short blinks, N = wakes counted in SRAM */
	      led_blink(100, 150, stop_wakes > 10 ? 10 : stop_wakes);
	      HAL_Delay(50);                           /* debounce */
	      while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET) {}
	      HAL_Delay(50);
	  #endif
	      /* USER CODE END 3 */
        }
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void led_blink(uint32_t on_ms, uint32_t off_ms, uint32_t n)
{
  while (n--) {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    HAL_Delay(on_ms);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_Delay(off_ms);
  }
}

/* Clock trap: PB1 carries three pulses
   w1 = fixed loop at HSI (16 MHz), w2 = SystemClock_Config() duration,
   w3 = the identical loop after the PLL is locked (84 MHz).
   Ratio = w1 / w3. */
static void trap_loop(void)
{
  for (volatile uint32_t i = 0; i < TRAP_LOOP_N; i++) { __NOP(); }
}

static void clock_trap_probe(void)
{
  MARK_TRAP_HI(); trap_loop();          MARK_TRAP_LO();   /* w1 */
  for (volatile int g = 0; g < 20; g++) { __NOP(); }
  MARK_TRAP_HI(); SystemClock_Config(); MARK_TRAP_LO();   /* w2 */
  for (volatile int g = 0; g < 20; g++) { __NOP(); }
  MARK_TRAP_HI(); trap_loop();          MARK_TRAP_LO();   /* w3 */
}

static void enter_stop(void)
{
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  MARK_WAKE_LO();
  MARK_TRAP_LO();

#if STOP_USE_FPDS
  PWR->CR |=  PWR_CR_FPDS;
#else
  PWR->CR &= ~PWR_CR_FPDS;
#endif

  HAL_SuspendTick();                     /* SysTick would otherwise wake us every 1 ms */
  __disable_irq();                       /* WFI still wakes on a pending IRQ; no lost-wake race */
  __HAL_GPIO_EXTI_CLEAR_IT(B1_Pin);
  HAL_NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
  in_stop = 1;
#if STOP_USE_LPDS
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#else
  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
#endif
  __enable_irq();                        /* the EXTI ISR runs here */
  HAL_ResumeTick();
}

static void enter_standby(void)
{
  HAL_SuspendTick();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  MARK_WAKE_LO();
  MARK_TRAP_LO();
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);     /* clear WUF before arming the pin */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);   /* PA0, rising edge */
  HAL_PWR_EnterSTANDBYMode();
  while (1) {}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == B1_Pin && in_stop) {
    MARK_WAKE_HI();                      /* first thing in the ISR: scope marker */
    in_stop = 0;
    stop_wakes++;                        /* SRAM counter; bounce while awake is ignored */
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
