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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "tm1637.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum BUTTONS
{
	BUTTON_UP = 0,
	BUTTON_DOWN,
	BUTTON_MODE,
	BUTTON_START,
	BUTTON_DBG,
	BUTTON_MAX
};

enum BUTTON_STATE
{
	BS_RELEASED,
	BS_PRESSED,
	BS_HOLD
};

typedef struct but
{
	uint8_t state;
	uint32_t time_pressed;
} button;

enum LEDS
{
	LED_DBG,
	LED_STARTED,
	LED_MODE,
	LED_MAX
};

enum LED_STATE
{
	LS_ON,
	LS_BLINK,
	LS_OFF
};

typedef struct
{
	uint8_t state;
	uint32_t time_change;
} led;

typedef enum {
    MODE_DIRECT = 0,
    MODE_TIMED
} app_mode_t;

typedef enum {
    EDIT_NONE = 0,
    EDIT_TIME,
    EDIT_SPEED
} edit_state_t;

typedef enum {
    PHASE_IDLE = 0,
    PHASE_UP,
    PHASE_DOWN
} timed_phase_t;

typedef struct {
    bool pressed;
    bool held;
    bool repeat;
} button_event_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_MS     20
#define BUTTON_HOLD_MS         800
#define BUTTON_REPEAT_DELAY_MS 500   // пауза до автоповтору
#define BUTTON_REPEAT_MS       120   // інтервал автоповтору
#define BLINK_PERIOD_MS        350

#define SPEED_MIN            1
#define SPEED_MAX            9999
#define SPEED_STEP           1
#define TIME_MIN_SEC         1
#define TIME_MAX_SEC         9999
#define TIME_STEP_SEC        1
#define DELAY 5

#define LED_RX_HOLD_MS       500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static app_mode_t    app_mode = MODE_DIRECT;
static edit_state_t  edit_state = EDIT_NONE;
static timed_phase_t timed_phase = PHASE_IDLE;
static bool          timed_running = false;

static int16_t  speed_value = 10;
static uint16_t time_value_100ms = 5;

static uint32_t last_blink_ms = 0;
static bool     blink_on = true;

/* для антидребезгу + автоповтор */
static uint32_t btn_ts[BUTTON_MAX]        = {0};
static uint32_t btn_hold_ts[BUTTON_MAX]   = {0};
static uint32_t btn_repeat_ts[BUTTON_MAX] = {0};
static bool     btn_state[BUTTON_MAX]     = {0};

/* індикація RX */
static uint32_t last_can_rx_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void app_tick(void);
static void handle_buttons(void);
static button_event_t check_button(enum BUTTONS b);
static void apply_mode_short_press(void);
static void cycle_edit_state(void);
static void start_stop_timed(void);
static void update_displays(void);
static void update_leds(void);
static void timed_fsm_tick(void);
static void poll_can_rx(void);

static void send_speed(int16_t speed);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
tm1637_t speedDisplay,timeDisplay;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  uart1_interrupt();
  CAN_Reconfigure_And_Start();

	speedDisplay.gpio_clk = H_SCK_GPIO_Port;
	speedDisplay.gpio_dat = H_SIO_GPIO_Port;
	speedDisplay.pin_clk = H_SCK_Pin;
	speedDisplay.pin_dat = H_SIO_Pin;
	speedDisplay.seg_cnt = 4;

	timeDisplay.gpio_clk = V_SCK_GPIO_Port;
	timeDisplay.gpio_dat = V_SIO_GPIO_Port;
	timeDisplay.pin_clk = V_SCK_Pin;
	timeDisplay.pin_dat = V_SIO_Pin;
	timeDisplay.seg_cnt = 4;

	tm1637_init(&speedDisplay);
	tm1637_init(&timeDisplay);
	tm1637_brightness(&speedDisplay, 8);
	tm1637_brightness(&timeDisplay, 8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("program start\n");
  while (1)
  {
	  HAL_Delay(DELAY);
	  if (hcan.State == HAL_CAN_STATE_ERROR)
	  {
		  CAN_Reconfigure_And_Start();
		  HAL_Delay(200);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  app_tick();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
static button_event_t check_button(enum BUTTONS b)
{
    button_event_t ev = {0};
    static const uint16_t pins[] = {BTN_UP_Pin, BTN_DOWN_Pin, BTN_MODE_Pin, BTN_START_Pin, BTN_DBG_Pin};
    bool level = (HAL_GPIO_ReadPin(GPIOA, pins[b]) == GPIO_PIN_RESET); // pull-up: натиснуто = 0
    uint32_t now = HAL_GetTick();

    if (level && !btn_state[b] && (now - btn_ts[b] > BUTTON_DEBOUNCE_MS)) {
        btn_state[b]     = true;
        btn_ts[b]        = now;
        btn_hold_ts[b]   = now;
        btn_repeat_ts[b] = now;
        ev.pressed = true;
    } else if (level && btn_state[b]) {
        if (!ev.pressed && (now - btn_hold_ts[b] > BUTTON_HOLD_MS)) {
            ev.held = true;
            btn_hold_ts[b] = now + 1000000; // щоб не багаторазово
        }
        if ((now - btn_ts[b] >= BUTTON_REPEAT_DELAY_MS) &&
            (now - btn_repeat_ts[b] >= BUTTON_REPEAT_MS)) {
            ev.repeat = true;
            btn_repeat_ts[b] = now;
        }
    } else if (!level && btn_state[b]) {
        btn_state[b] = false;
        btn_ts[b] = now;
    }
    return ev;
}

static void apply_mode_short_press(void)
{
    if (edit_state != EDIT_NONE) return; // у редагуванні коротке не міняє режим
    app_mode = (app_mode == MODE_DIRECT) ? MODE_TIMED : MODE_DIRECT;
    timed_running = false;
    timed_phase = PHASE_IDLE;
}

static void cycle_edit_state(void)
{
    if (edit_state == EDIT_NONE) {
        edit_state = EDIT_TIME;
    } else if (edit_state == EDIT_TIME) {
        edit_state = EDIT_SPEED;
    } else {
        edit_state = EDIT_NONE;
    }
}

static void start_stop_timed(void)
{
    if (app_mode != MODE_TIMED || edit_state != EDIT_NONE) return;
    timed_running = !timed_running;
    if (timed_running) {
        timed_phase = PHASE_UP;
        send_speed(speed_value);
        last_blink_ms = HAL_GetTick();
    } else {
        timed_phase = PHASE_IDLE;
    }
}

static void timed_fsm_tick(void)
{
    if (!timed_running) return;
    uint32_t now = HAL_GetTick();

    switch (timed_phase) {
    case PHASE_UP:
        if (now - last_blink_ms >= time_value_100ms * 100UL) {
            timed_phase = PHASE_DOWN;
            send_speed(-speed_value);
            last_blink_ms = now;
        }
        break;
    case PHASE_DOWN:
        if (now - last_blink_ms >= time_value_100ms * 1000UL) {
            timed_phase = PHASE_IDLE;
            timed_running = false;
        }
        break;
    default:
        break;
    }
}

static void handle_buttons(void)
{
    button_event_t ev_mode  = check_button(BUTTON_MODE);
    button_event_t ev_up    = check_button(BUTTON_UP);
    button_event_t ev_down  = check_button(BUTTON_DOWN);
    button_event_t ev_start = check_button(BUTTON_START);

    const bool up_act   = ev_up.pressed   || ev_up.repeat;
    const bool down_act = ev_down.pressed || ev_down.repeat;

    if (ev_mode.held) {
        cycle_edit_state();
    } else if (ev_mode.pressed) {
        apply_mode_short_press();
    }

    if (edit_state == EDIT_TIME) {
        if (up_act   && time_value_100ms < TIME_MAX_SEC) time_value_100ms += TIME_STEP_SEC;
        if (down_act && time_value_100ms > TIME_MIN_SEC) time_value_100ms -= TIME_STEP_SEC;
    } else if (edit_state == EDIT_SPEED) {
        if (up_act   && speed_value < SPEED_MAX) speed_value += SPEED_STEP;
        if (down_act && speed_value > SPEED_MIN) speed_value -= SPEED_STEP;
    } else { // EDIT_NONE
        if (app_mode == MODE_DIRECT) {
            if (up_act   && speed_value < SPEED_MAX) speed_value += SPEED_STEP;
            if (down_act && speed_value > SPEED_MIN) speed_value -= SPEED_STEP;
            if (ev_up.pressed || ev_up.repeat)   send_speed(speed_value);
            if (ev_down.pressed || ev_down.repeat) send_speed(-speed_value);
        }
    }

    if (ev_start.pressed) {
        start_stop_timed();
    }
}

static void update_displays(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_blink_ms >= BLINK_PERIOD_MS) {
        last_blink_ms = now;
        blink_on = !blink_on;
    }

    bool blink_time  = (edit_state == EDIT_TIME);
    bool blink_speed = (edit_state == EDIT_SPEED);

    if (blink_speed && !blink_on) {
        tm1637_disp_clear(&speedDisplay);
    } else {
        tm1637_disp_printf(&speedDisplay, "%4u", speed_value);
    }

    if (blink_time && !blink_on) {
        tm1637_disp_clear(&timeDisplay);
    } else {
        tm1637_disp_printf(&timeDisplay, "%4u", time_value_100ms);
    }
}

static void update_leds(void)
{
    HAL_GPIO_WritePin(LED_MODE_GPIO_Port, LED_MODE_Pin,
                      (app_mode == MODE_TIMED) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_START_GPIO_Port, LED_START_Pin,
                      (timed_running) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    uint32_t now = HAL_GetTick();
    GPIO_PinState dbg = (now - last_can_rx_ms <= LED_RX_HOLD_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, dbg);
}

static void app_tick(void)
{
    handle_buttons();
    timed_fsm_tick();
    poll_can_rx();
    update_displays();
    update_leds();
}

void send_speed(int16_t speed)
{
	uint8_t data[8];
	data[0]=0x01;
	data[1]=0x20;
	data[2]=0x0D;
	data[3]=0x03;
	uint16_t v = speed;
	data[4]=v & 0xFF;
	data[5]=(v>>8)&0xFF;
	if(speed < 0)
	{data[6] = 0xFF; data[7]=0xFF;} else
	{data[6] = 0; data[7]=0;}
	CAN_Send(0x201,data,8);
	CAN_Send(0x202,data,8);
}

static void poll_can_rx(void)
{
    CAN_RxHeaderTypeDef hdr;
    uint8_t data[CAN_MESSAGE_SIZE];
    uint32_t now = HAL_GetTick();

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK) {
            last_can_rx_ms = now;
        } else {
            break;
        }
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
