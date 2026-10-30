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
#include <stdlib.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define LINK_START        0xA5u
#define LINK_MAX_PAYLOAD  8u
#define MSG_SET_PULSE     0x01u
#define MSG_ACK           0x81u
#define MSG_TELEMETRY     0x82u
#define RING_SIZE         64u

/* debug pin PA8 (D7) for the scope */
#define DBG_HIGH()  (GPIOA->BSRR = (1u << 8))
#define DBG_LOW()   (GPIOA->BSRR = (1u << (8u + 16u)))

/* printf -> USART2 (ST-LINK virtual COM port) */
int _write(int fd, char *ptr, int len)
{
    (void)fd;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* CRC-16/MODBUS */
static uint16_t crc16_update(uint16_t crc, uint8_t b)
{
    crc ^= b;
    for (int k = 0; k < 8; k++)
        crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
    return crc;
}

static uint16_t crc16_buf(const uint8_t *d, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) crc = crc16_update(crc, *d++);
    return crc;
}

/* Build a frame into out[]; returns total length (LEN + 4). */
static uint8_t link_build(uint8_t *out, const uint8_t *payload, uint8_t len)
{
    out[0] = LINK_START;
    out[1] = len;
    memcpy(&out[2], payload, len);
    uint16_t crc = crc16_buf(&out[1], (uint32_t)len + 1u);   /* LEN + payload */
    out[2 + len] = (uint8_t)(crc & 0xFFu);                   /* low byte first */
    out[3 + len] = (uint8_t)(crc >> 8);
    return (uint8_t)(len + 4u);
}

/* Receiver state machine: WAIT_START -> LEN -> PAYLOAD -> CRC -> check */
typedef enum { RX_WAIT_START, RX_READ_LEN, RX_READ_PAYLOAD,
               RX_READ_CRC_LO, RX_READ_CRC_HI } rx_state_t;

typedef struct {
    rx_state_t state;
    uint8_t    len, idx, crc_lo;
    uint8_t    buf[LINK_MAX_PAYLOAD];
    uint16_t   crc;
    uint32_t   n_ok, n_crc_err, n_len_err, n_aborted;
} link_rx_t;

/* returns 1 when a complete, CRC-correct frame is in r->buf (r->len bytes) */
static int link_rx_feed(link_rx_t *r, uint8_t b)
{
    switch (r->state) {
    case RX_WAIT_START:
        if (b == LINK_START) r->state = RX_READ_LEN;
        break;

    case RX_READ_LEN:
        if (b >= 1u && b <= LINK_MAX_PAYLOAD) {
            r->len = b;
            r->idx = 0;
            r->crc = crc16_update(0xFFFFu, b);        /* CRC covers LEN too */
            r->state = RX_READ_PAYLOAD;
        } else if (b == LINK_START) {
            /* the previous 0xA5 was not a real start, this one may be: stay */
        } else {
            r->n_len_err++;                            /* impossible LEN: give up */
            r->state = RX_WAIT_START;
        }
        break;

    case RX_READ_PAYLOAD:
        r->buf[r->idx++] = b;
        r->crc = crc16_update(r->crc, b);
        if (r->idx == r->len) r->state = RX_READ_CRC_LO;
        break;

    case RX_READ_CRC_LO:
        r->crc_lo = b;
        r->state = RX_READ_CRC_HI;
        break;

    case RX_READ_CRC_HI:
        r->state = RX_WAIT_START;
        if ((uint16_t)(r->crc_lo | ((uint16_t)b << 8)) == r->crc) {
            r->n_ok++;
            return 1;
        }
        r->n_crc_err++;
        break;
    }
    return 0;
}

/* the line went idle in the middle of a frame: give up, wait for next 0xA5 */
static void link_rx_abort(link_rx_t *r)
{
    if (r->state != RX_WAIT_START) {
        r->n_aborted++;
        r->state = RX_WAIT_START;
    }
}

/* RX: DMA ring + IDLE line */
static uint8_t           rx_ring[RING_SIZE];
static uint32_t          rx_tail;                 /* where we stopped reading   */
static link_rx_t         lrx;
static volatile uint8_t  idle_pending;            /* set by the IDLE interrupt  */
static volatile uint32_t idle_head;               /* ring head at that moment   */
static volatile uint8_t  rx_restart;              /* set by HAL error callback  */
static uint32_t          n_rx_restarts;

/* start (or restart) the circular DMA receive and switch on the IDLE interrupt */
static void link_rx_start(void)
{
    lrx.state = RX_WAIT_START;
    rx_tail = 0;
    HAL_UART_Receive_DMA(&huart1, rx_ring, RING_SIZE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);      /* CubeMX has no tick box */
}

/* called from USART1_IRQHandler */
void link_idle_isr(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);               /* read SR, then DR */
        idle_head    = (RING_SIZE - hdma_usart1_rx.Instance->NDTR) % RING_SIZE;
        idle_pending = 1;
    }
}

/* Any USART error while DMA is running makes HAL abort the receive stream.
 * Flag it, and the main loop restarts the ring. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) rx_restart = 1;
}

/* Call every pass of the main loop. Hands each good frame to on_frame(). */
static void link_poll_rx(void (*on_frame)(const uint8_t *p, uint8_t len))
{
    if (rx_restart) {
        rx_restart = 0;
        if (huart1.RxState == HAL_UART_STATE_READY) {
            link_rx_start();
            n_rx_restarts++;
        }
    }

    uint8_t  was_idle = idle_pending;
    uint32_t ih = 0;
    if (was_idle) { idle_pending = 0; ih = idle_head; }

    uint32_t head = (RING_SIZE - hdma_usart1_rx.Instance->NDTR) % RING_SIZE;  /* read NDTR once */

    while (rx_tail != head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1u) % RING_SIZE;
        if (link_rx_feed(&lrx, b)) on_frame(lrx.buf, lrx.len);
    }

    /* idle, and nothing new has arrived since: a half-received frame is dead */
    if (was_idle && head == ih) link_rx_abort(&lrx);
}

/* TX: DMA, one frame at a time */
static uint8_t tx_buf[LINK_MAX_PAYLOAD + 4u];

static void tx_frame(const uint8_t *payload, uint8_t len)
{
    uint32_t t0 = HAL_GetTick();
    while (huart1.gState != HAL_UART_STATE_READY) {      /* earlier frame still leaving */
        if (HAL_GetTick() - t0 > 5u) { HAL_UART_AbortTransmit(&huart1); break; }
    }
    uint8_t n = link_build(tx_buf, payload, len);
    HAL_UART_Transmit_DMA(&huart1, tx_buf, n);
}

/* Self test: the three example frames from the lecture slide */
static void link_selftest(void)
{
    static const uint8_t set_pl[] = { MSG_SET_PULSE, 0x11, 0xDC, 0x05 };
    static const uint8_t ack_pl[] = { MSG_ACK, 0x11, 0x00 };
    static const uint8_t tel_pl[] = { MSG_TELEMETRY, 0x3C, 0xDC, 0x05 };
    static const uint8_t set_ex[] = { 0xA5,0x04,0x01,0x11,0xDC,0x05,0x1C,0xFA };
    static const uint8_t ack_ex[] = { 0xA5,0x03,0x81,0x11,0x00,0x5C,0x18 };
    static const uint8_t tel_ex[] = { 0xA5,0x04,0x82,0x3C,0xDC,0x05,0xA5,0x77 };
    uint8_t f[LINK_MAX_PAYLOAD + 4u];
    int ok = 1;
    ok &= (link_build(f, set_pl, 4) == 8) && memcmp(f, set_ex, 8) == 0;
    ok &= (link_build(f, ack_pl, 3) == 7) && memcmp(f, ack_ex, 7) == 0;
    ok &= (link_build(f, tel_pl, 4) == 8) && memcmp(f, tel_ex, 8) == 0;
    printf("CRC self test: %s\r\n", ok ? "PASS" : "FAIL");
}
/* ---------- state of the node ---------- */
static volatile uint8_t  tele_due;          /* set every 20 ms by the TIM3 update interrupt */
static volatile uint16_t measured_us;       /* last pulse width captured on PA0 (us)        */
static uint8_t  tele_count;
static uint8_t  have_seq, last_seq, last_status;
static uint32_t n_cmds, n_applied, n_dup, n_range;

/* on_frame() only fills these; the main loop prints them, so printf never delays an ACK */
static uint8_t  log_flag, log_seq, log_kind;      /* kind: 0 applied, 1 repeat, 2 out of range */
static uint16_t log_pulse;

/* TIM3 update event = end of every 20 ms servo period */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) tele_due = 1;
}

/* TIM2 PWM-input: CCR2 is the pulse width in ticks, 1 tick = 1 us (PSC 83) */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        measured_us = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
}

/* one good frame has arrived */
static void on_frame(const uint8_t *p, uint8_t len)
{
    if (p[0] != MSG_SET_PULSE || len != 4u) return;

    DBG_HIGH();                                        /* scope: parse + check + write CCR */
    uint8_t  seq   = p[1];
    uint16_t pulse = (uint16_t)(p[2] | ((uint16_t)p[3] << 8));   /* low byte first */
    uint8_t  status, kind;

    n_cmds++;
    if (have_seq && seq == last_seq) {                 /* repeated SEQ: ACK again, do not apply */
        status = last_status;
        kind   = 1;
        n_dup++;
    } else {
        if (pulse >= 1000u && pulse <= 2000u) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);   /* applied */
            status = 0; kind = 0; n_applied++;
        } else {
            status = 1; kind = 2; n_range++;           /* servo does not move */
        }
        have_seq = 1; last_seq = seq; last_status = status;
    }
    DBG_LOW();

    uint8_t ack[3] = { MSG_ACK, seq, status };
    tx_frame(ack, 3);                                  /* queues behind a TELEMETRY frame if one is leaving */

    log_flag = 1; log_seq = seq; log_pulse = pulse; log_kind = kind;
}

static void send_telemetry(void)
{
    uint16_t m = measured_us;
    uint8_t t[4] = { MSG_TELEMETRY, tele_count++, (uint8_t)(m & 0xFFu), (uint8_t)(m >> 8) };
    tx_frame(t, 4);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  setvbuf(stdout, NULL, _IONBF, 0);
  printf("\r\nBOARD A ready\r\n");
  link_selftest();

  link_rx_start();                                   /* circular DMA ring + IDLE interrupt */

  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);        /* PA0: period capture   */
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);        /* PA0: width capture    */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);          /* PA6: servo pulse      */
  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);        /* 20 ms telemetry tick  */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    link_poll_rx(on_frame);

	    if (tele_due) {
	      tele_due = 0;
	      send_telemetry();
	    }

	    if (log_flag) {
	      log_flag = 0;
	      printf("seq=%3u  SET %4u us  %s\r\n", (unsigned)log_seq, (unsigned)log_pulse,
	             log_kind == 0 ? "applied" : (log_kind == 1 ? "repeat, ACKed again" : "OUT OF RANGE, status 01"));
	      if (n_cmds % 100u == 0u)
	        printf("A stats: cmds=%lu applied=%lu repeat=%lu range=%lu  crc_err=%lu len_err=%lu idle_abort=%lu rx_restart=%lu  measured=%u us\r\n",
	               (unsigned long)n_cmds, (unsigned long)n_applied, (unsigned long)n_dup, (unsigned long)n_range,
	               (unsigned long)lrx.n_crc_err, (unsigned long)lrx.n_len_err, (unsigned long)lrx.n_aborted,
	               (unsigned long)n_rx_restarts, (unsigned)measured_us);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
