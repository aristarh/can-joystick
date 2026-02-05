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

typedef enum
{
	BS_RELEASED,
	BS_PRESSED,
	BS_HOLD,
	BS_NONE
} BUTTON_STATE;

typedef struct but
{
	bool level;
	bool pressed;
	bool holded;
	uint32_t time_changed;
} button;

static button buttons[BUTTON_MAX];

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
	MODE_REPEAT,
    MODE_CHANGE
} work_modes;

typedef enum {
	CHANGE_NONE,
	CHANGE_SPEED,
	CHANGE_TIME
} change_modes;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUTTON_DEBOUNCE_MS     20
#define BUTTON_HOLD_MS         1000
#define BUTTON_REPEAT_MS       250   // інтервал автоповтору
#define BLINK_PERIOD_MS        350

#define VAL_MIN            1
#define VAL_MAX            9999
#define VAL_STEP           1
#define VAL_BIG_STEP       20
#define DELAY 5

#define LED_RX_HOLD_MS       50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static work_modes app_mode = MODE_DIRECT;
static change_modes change_mode = CHANGE_NONE;

static bool repeat_started = false;
static bool repeat_up = true;
static uint32_t repeat_time = 0;

static uint16_t  speed_value = 10;
static uint16_t time_value_100ms = 5;

static uint32_t last_disp_blink_ms = 0;
static bool     disp_blink_on = true;

/* індикація RX */
static uint32_t last_can_rx_ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static GPIO_PinState button_state(enum BUTTONS b);
static void handle_buttons(void);
static BUTTON_STATE check_button(enum BUTTONS b);
static void mode_short_press(void);
static void mode_long_press(void);
static void start_stop_timed(void);
static void update_displays(void);
static void update_leds(void);
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
	    handle_buttons();
	    poll_can_rx();
	    update_displays();
	    update_leds();
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

static GPIO_PinState button_state(enum BUTTONS b)
{
	switch (b){
	case BUTTON_UP: return HAL_GPIO_ReadPin(BTN_UP_GPIO_Port, BTN_UP_Pin); break;
	case BUTTON_DOWN: return HAL_GPIO_ReadPin(BTN_DOWN_GPIO_Port, BTN_DOWN_Pin); break;
	case BUTTON_DBG: return HAL_GPIO_ReadPin(BTN_DBG_GPIO_Port, BTN_DBG_Pin); break;
	case BUTTON_MODE: return HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin); break;
	case BUTTON_START: return HAL_GPIO_ReadPin(BTN_START_GPIO_Port, BTN_START_Pin); break;
	default:
	}
	return GPIO_PIN_RESET;
}

static BUTTON_STATE check_button(enum BUTTONS b)
{
	bool level = (button_state(b) == GPIO_PIN_RESET); // Active LOW
	uint32_t now = HAL_GetTick();
	uint32_t duration = now - buttons[b].time_changed;

	if (level != buttons[b].level)
	{
		buttons[b].time_changed = now;
		buttons[b].level = level;
		return BS_NONE;
	}

	if (duration > BUTTON_DEBOUNCE_MS)
	{
		if (level && !buttons[b].pressed)
		{
			buttons[b].pressed = true;
			return BS_PRESSED;
		}
		if (level && buttons[b].pressed)
		{
			uint16_t time = (buttons[b].holded)? BUTTON_REPEAT_MS:BUTTON_HOLD_MS;

			if (duration > time)
			{
				buttons[b].time_changed = now;
				buttons[b].holded = true;
				return BS_HOLD;
			}
		}
		if (!level && buttons[b].pressed)
		{
			buttons[b].pressed = false;
			buttons[b].holded = false;
			return BS_RELEASED;
		}
	}
	return BS_NONE;
}

static void mode_short_press(void)
{
	if (repeat_started) return;
    if (app_mode == MODE_CHANGE)
    {
    	if (change_mode == CHANGE_TIME)
    		change_mode = CHANGE_SPEED;
    	else change_mode = CHANGE_TIME;
    } else app_mode = (app_mode == MODE_DIRECT) ? MODE_REPEAT : MODE_DIRECT;
}

static void mode_long_press(void)
{
	if (repeat_started) return;
	if (app_mode != MODE_CHANGE)
	{
		app_mode = MODE_CHANGE;
		change_mode = CHANGE_TIME;
	} else
	{
		app_mode = MODE_DIRECT;
		change_mode = CHANGE_NONE;
	}
}

static void handle_buttons(void)
{
    uint32_t now = HAL_GetTick();
    BUTTON_STATE ms = check_button(BUTTON_MODE);
    BUTTON_STATE up = check_button(BUTTON_UP);
    BUTTON_STATE dn = check_button(BUTTON_DOWN);
    if (ms == BS_HOLD) mode_long_press();
    else if (ms == BS_PRESSED) mode_short_press();
    if (app_mode == MODE_REPEAT)
    {
    	BUTTON_STATE st = check_button(BUTTON_START);
    	if (st == BS_RELEASED)
    	{
    		repeat_started = !repeat_started;
    		repeat_up = true;
    		repeat_time = now;
    	}
    }

    if (app_mode == MODE_CHANGE)
    {
    	int v = 0;
    	if (up == BS_HOLD) v+= VAL_BIG_STEP; else if (up == BS_PRESSED) v+= VAL_STEP; else
		if (dn == BS_HOLD) v-= VAL_BIG_STEP; else if (dn == BS_PRESSED) v-= VAL_STEP;
    	if (v)
    	{
    		uint16_t *vc = 0;

    		if (change_mode == CHANGE_SPEED)
    			vc = &speed_value;
    		else
    			vc = &time_value_100ms;
    		int val = *vc + v;
			if (val > VAL_MAX) val = VAL_MAX; else
			if (val < VAL_MIN) val = VAL_MIN;
			*vc = val;
    	}
    } else if (app_mode == MODE_DIRECT)
    {
    	if (buttons[BUTTON_UP].pressed) send_speed(speed_value);
    	else if (buttons[BUTTON_DOWN].pressed) send_speed(-speed_value);
    	else send_speed(0);
    } else if (app_mode == MODE_REPEAT)
    {
    	if (repeat_started)
    	{
    		start_stop_timed();
    	}
    }
}

static void start_stop_timed(void)
{
    uint32_t now = HAL_GetTick();
    if (now - repeat_time > time_value_100ms * 100)
    {
    	repeat_up = !repeat_up;
    	repeat_time = now;
    }
    int16_t v = (repeat_up)? -speed_value : speed_value;
    send_speed(v);
}

static void update_displays(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_disp_blink_ms >= BLINK_PERIOD_MS) {
        last_disp_blink_ms = now;
        disp_blink_on = !disp_blink_on;
    }

    bool blink_time  = (change_mode == CHANGE_TIME);
    bool blink_speed = (change_mode == CHANGE_SPEED);

    if (blink_speed && !disp_blink_on) {
        tm1637_disp_clear(&speedDisplay);
    } else {
        tm1637_disp_printf(&speedDisplay, "%4u", speed_value);
    }

    if (blink_time && !disp_blink_on) {
        tm1637_disp_clear(&timeDisplay);
    } else {
        tm1637_disp_printf(&timeDisplay, "%4u", time_value_100ms);
    }
}

static void update_leds(void)
{
    HAL_GPIO_WritePin(LED_MODE_GPIO_Port, LED_MODE_Pin,
                      (app_mode == MODE_REPEAT) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_START_GPIO_Port, LED_START_Pin,
                      (repeat_started) ? GPIO_PIN_RESET : GPIO_PIN_SET);

    uint32_t now = HAL_GetTick();
    GPIO_PinState dbg = (now - last_can_rx_ms <= LED_RX_HOLD_MS) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LED_DBG_GPIO_Port, LED_DBG_Pin, dbg);
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

    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK)
        {
        	uint8_t addr = hdr.StdId&0x7F;
        	uint16_t type = hdr.StdId-addr;
			if (type == 0x700)
			{
				if (hdr.DLC == 1)
				{
					if (data[0] == 0x7F)
					{
						//motor not inited
						uint8_t d[8];
						d[0] = 0x01; // Start Node
						d[1] = addr;
						CAN_Send(0x00, d, 2);
					}
				}
			}
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
