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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
enum BUTTON
{
	BUTTON_UP =0,
	BUTTON_DOWN,
	BUTTON_LEFT,
	BUTTON_RIGHT,
	BUTTON_MAX
} ;
#define TIME_SEND 300
#define SPEED_V 40
#define SPEED_H 300
#define DELAY 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void buttons();
void getAzimuths();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
tm1637_t hDisplay,vDisplay;
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
//  CAN_Start_Interrupt(&hcan);
  CAN_Reconfigure_And_Start();

	hDisplay.gpio_clk = H_SCK_GPIO_Port;
	hDisplay.gpio_dat = H_SIO_GPIO_Port;
	hDisplay.pin_clk = H_SCK_Pin;
	hDisplay.pin_dat = H_SIO_Pin;
	hDisplay.seg_cnt = 4;

	vDisplay.gpio_clk = V_SCK_GPIO_Port;
	vDisplay.gpio_dat = V_SIO_GPIO_Port;
	vDisplay.pin_clk = V_SCK_Pin;
	vDisplay.pin_dat = V_SIO_Pin;
	vDisplay.seg_cnt = 4;

	tm1637_init(&hDisplay);
	tm1637_init(&vDisplay);
	tm1637_brightness(&hDisplay, 8);
	tm1637_brightness(&vDisplay, 8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("program start\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	HAL_Delay(DELAY);
	if (hcan.State == HAL_CAN_STATE_ERROR)
	{
		CAN_Reconfigure_And_Start();
		HAL_Delay(200);
	}

	buttons();
	getAzimuths();

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
void buttons()
{

	static GPIO_PinState B_last_state[BUTTON_MAX];
	static GPIO_PinState B_state[BUTTON_MAX];
	static  uint16_t count = 0;
	static int16_t speed_h,speed_v;

	count += DELAY;
	B_state[BUTTON_UP] = HAL_GPIO_ReadPin(B_UP_GPIO_Port, B_UP_Pin);
	B_state[BUTTON_DOWN] = HAL_GPIO_ReadPin(B_DOWN_GPIO_Port, B_DOWN_Pin);
	B_state[BUTTON_LEFT] = HAL_GPIO_ReadPin(B_LEFT_GPIO_Port, B_LEFT_Pin);
	B_state[BUTTON_RIGHT] = HAL_GPIO_ReadPin(B_RIGHT_GPIO_Port, B_RIGHT_Pin);


	if (B_state[BUTTON_UP] != B_last_state[BUTTON_UP])
	{
		printf("up %i\n",B_state[BUTTON_UP]);
		B_last_state[BUTTON_UP] = B_state[BUTTON_UP];
		count = 0xFFFF;
	}
	if (B_state[BUTTON_DOWN] != B_last_state[BUTTON_DOWN])
	{
		printf("down %i\n",B_state[BUTTON_DOWN]);
		B_last_state[BUTTON_DOWN] = B_state[BUTTON_DOWN];
		count = 0xFFFF;
	}
	if (B_state[BUTTON_LEFT] != B_last_state[BUTTON_LEFT])
	{
		printf("left %i\n",B_state[BUTTON_LEFT]);
		B_last_state[BUTTON_LEFT] = B_state[BUTTON_LEFT];
		count = 0xFFFF;
	}
	if (B_state[BUTTON_RIGHT] != B_last_state[BUTTON_RIGHT])
	{
		printf("right %i\n",B_state[BUTTON_RIGHT]);
		B_last_state[BUTTON_RIGHT] = B_state[BUTTON_RIGHT];
		count = 0xFFFF;
	}
	if (count > TIME_SEND)
	{
		if (B_state[BUTTON_LEFT] == B_state[BUTTON_RIGHT])
		{
			speed_h = 0;
		} else if (B_state[BUTTON_LEFT] == GPIO_PIN_SET)
		{
			speed_h = -SPEED_H;
		} else if (B_state[BUTTON_RIGHT] == GPIO_PIN_SET)
		{
			speed_h = SPEED_H;
		}
		if (B_state[BUTTON_UP] == B_state[BUTTON_DOWN])
		{
			speed_v = (0);
		} else if (B_state[BUTTON_DOWN] == GPIO_PIN_SET)
		{
			speed_v = (-SPEED_V);
		} else if (B_state[BUTTON_UP] == GPIO_PIN_SET)
		{
			speed_v = (SPEED_V);
		}
		count = 0;
		send_speed(speed_h,speed_v);
	}
}

void getAzimuths()
{
	static CAN_RxHeaderTypeDef header;
	static uint8_t data[CAN_MESSAGE_SIZE];
	static uint16_t hAzimuth = 100,vAzimuth=200;
	if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))
	{
		HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &header, data);
		printf("get msg %i data[0] %i \n",header.StdId,data[0]);
		if (header.StdId == 0x185)
		{
//			printf("0 get message id %i",header.StdId);
//			HAL_UART_Transmit(&huart1, data, header.DLC, 0xFF);
//			printf("\n");
			if (header.DLC == 5)
			{
				if (data[0] &= 0x01)
				{
					tm1637_brightness(&hDisplay, 8);
					tm1637_disp_str(&hDisplay,"ERR");

				} else
				{
					hAzimuth = (data[2]<<8)+data[1];
					tm1637_brightness(&hDisplay, 8);
					tm1637_disp_printf(&hDisplay,"%i", hAzimuth);


				} if (data[0] &= 0x02)
				{
					tm1637_brightness(&vDisplay, 8);
					tm1637_disp_str(&vDisplay,"ERR");
				} else
				{
					vAzimuth = (data[4]<<8)+data[3];
					tm1637_brightness(&vDisplay, 8);
					tm1637_disp_printf(&vDisplay,"%i", vAzimuth);
				}
			}
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
