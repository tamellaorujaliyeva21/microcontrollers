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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOF_BYTE          0xA5u
#define TYPE_RSSI         0x01u
#define RSSI_PAYLOAD_LEN  3u
#define MAX_PAYLOAD       16u
#define WIN_SIZE          20u
#define Q_SIZE            32u      /* must be a power of 2 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct { int8_t rssi; int8_t tx_dbm; uint8_t seq; } reading_t;

static uint8_t rx_byte;                          /* 1-byte IT receive buffer */

static volatile reading_t q[Q_SIZE];             /* ISR -> main ring buffer */
static volatile uint8_t   q_head, q_tail;
static volatile uint32_t  bad_frames, q_overflow;

typedef enum { S_SOF, S_LEN, S_BODY } pstate_t;
static pstate_t pst = S_SOF;
static uint8_t  plen, pidx;
static uint8_t  pbody[MAX_PAYLOAD + 2];

static const uint8_t dist_x10[] = {5, 10, 20, 40, 80, 160};   /* distance in 0.1 m units */
#define N_DIST  (sizeof(dist_x10) / sizeof(dist_x10[0]))
static uint8_t dist_idx = 1;                                   /* start at 1.0 m */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t crc8(uint8_t crc, const uint8_t *d, uint8_t n)
{
    while (n--) {
        crc ^= *d++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void q_push(int8_t rssi, int8_t tx, uint8_t seq)   /* called from ISR */
{
    uint8_t next = (uint8_t)((q_head + 1u) & (Q_SIZE - 1u));
    if (next == q_tail) { q_overflow++; return; }
    q[q_head].rssi   = rssi;
    q[q_head].tx_dbm = tx;
    q[q_head].seq    = seq;
    q_head = next;
}

static int q_pop(reading_t *r)                             /* called from main */
{
    if (q_tail == q_head) return 0;
    r->rssi   = q[q_tail].rssi;
    r->tx_dbm = q[q_tail].tx_dbm;
    r->seq    = q[q_tail].seq;
    q_tail = (uint8_t)((q_tail + 1u) & (Q_SIZE - 1u));
    return 1;
}

/* Byte-wise parser: 0xA5 | LEN | TYPE | payload[LEN] | CRC8 */
static void parse_byte(uint8_t b)
{
    switch (pst) {
    case S_SOF:
        if (b == SOF_BYTE) pst = S_LEN;
        break;

    case S_LEN:
        if (b <= MAX_PAYLOAD) { plen = b; pidx = 0; pst = S_BODY; }
        else                  { pst = S_SOF; bad_frames++; }
        break;

    case S_BODY:
        pbody[pidx++] = b;
        if (pidx == (uint8_t)(plen + 2u)) {           /* TYPE + payload + CRC received */
            uint8_t crc = crc8(0, &plen, 1);
            crc = crc8(crc, pbody, (uint8_t)(plen + 1u));
            if (crc == pbody[plen + 1u] && pbody[0] == TYPE_RSSI && plen == RSSI_PAYLOAD_LEN) {
                q_push((int8_t)pbody[1], (int8_t)pbody[2], pbody[3]);
            } else {
                bad_frames++;
            }
            pst = S_SOF;
        }
        break;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        parse_byte(rx_byte);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);    /* re-arm after overrun/noise */
    }
}

/* Median x2 so a half-dB result stays an integer (even N -> mean of middle two) */
static int median_x2(const int8_t *src, uint8_t n)
{
    int8_t a[WIN_SIZE];
    memcpy(a, src, n);
    for (uint8_t i = 1; i < n; i++) {                 /* insertion sort */
        int8_t key = a[i];
        int8_t j = (int8_t)i - 1;
        while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
        a[j + 1] = key;
    }
    if (n & 1u) return 2 * a[n / 2];
    return a[n / 2 - 1] + a[n / 2];
}

static void print_median(int8_t tx, int m2)
{
    char buf[96];
    int a = (m2 < 0) ? -m2 : m2;
    unsigned d10 = dist_x10[dist_idx];
    int len = snprintf(buf, sizeof(buf),
                       "> d=%u.%u m   tx=%d dBm   n=%u   med=%s%d.%d\r\n",
                       d10 / 10u, d10 % 10u, tx, (unsigned)WIN_SIZE,
                       (m2 < 0) ? "-" : "", a / 2, (a & 1) ? 5 : 0);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);

    if (bad_frames) {                                  /* only shown if something went wrong */
        len = snprintf(buf, sizeof(buf), "  (bad frames so far: %lu)\r\n", (unsigned long)bad_frames);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);
    }
}

static void print_distance(void)
{
    char buf[48];
    unsigned d10 = dist_x10[dist_idx];
    int len = snprintf(buf, sizeof(buf), "distance set: %u.%u m\r\n", d10 / 10u, d10 % 10u);
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 50);
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  static const char banner[] = "LAB5A ready: median of 20 RSSI readings\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)banner, sizeof(banner) - 1, 50);
  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

  int8_t  win[WIN_SIZE];
  uint8_t win_n = 0;
  int8_t  win_tx = 0;

  print_distance();
  uint8_t  btn_prev = 0;
  uint32_t last_btn = 0;
  uint32_t settle_until = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /* Blue button (PC13, active low): next distance */
	  uint8_t btn = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET);
	  if (btn && !btn_prev && (HAL_GetTick() - last_btn) > 50) {
	      last_btn = HAL_GetTick();
	      dist_idx = (uint8_t)((dist_idx + 1u) % N_DIST);
	      win_n = 0;                                   /* start a fresh window */
	      settle_until = HAL_GetTick() + 3000;         /* ignore readings for 3 s: step back! */
	      print_distance();
	  }
	  btn_prev = btn;

	  reading_t r;
	  while (q_pop(&r)) {
	      if ((int32_t)(HAL_GetTick() - settle_until) < 0) continue;   /* settling after a button press */

	      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

	      /* TX power changed (sweep step): discard the partial window */
	      if (win_n > 0 && r.tx_dbm != win_tx) win_n = 0;
	      win_tx = r.tx_dbm;

	      win[win_n++] = r.rssi;
	      if (win_n == WIN_SIZE) {
	          print_median(win_tx, median_x2(win, WIN_SIZE));
	          win_n = 0;
	      }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Blue user button on PC13: input, active low (the Nucleo already has a pull-up) */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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
