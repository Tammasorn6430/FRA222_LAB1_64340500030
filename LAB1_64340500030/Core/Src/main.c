/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct PortPin {
	GPIO_TypeDef *PORT;
	uint16_t PIN;
};

struct PortPin R[4] = {
		{ GPIOA, GPIO_PIN_10 }, //A10
		{ GPIOC, GPIO_PIN_9 }, //C9
		{ GPIOB, GPIO_PIN_5 }, //B5
		{ GPIOB, GPIO_PIN_8 } //B8
};

struct PortPin L[4] = {
		{ GPIOA, GPIO_PIN_9 }, //A9
		{ GPIOC, GPIO_PIN_7 }, //C7
		{ GPIOB, GPIO_PIN_6 }, //B6
		{ GPIOA, GPIO_PIN_7 } //A7
};

uint16_t ButtonMatrix = 0;
uint16_t ButtonMatrix_L = 0;
int flag = 0;
int state = 0, state_L = 0;
int number = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadMatrixButton_1Row();
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		//rising edge
		if (ButtonMatrix_L == 0 && ButtonMatrix != 0) {
			flag = 1;
		} else {
			flag = 0;
		}
		ButtonMatrix_L = ButtonMatrix;

		//state transition
		switch (state) {
		case 0:
			//initial LED
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LED1 off
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //LED2 off
			number = 1111;
			if (flag) {
				if (ButtonMatrix == 512) {state = 1;} //6
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state = 0;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;} //0, ok, no value switches, backspace
				state_L = 0;
			}
			break;
		case 1:
			number = 6;
			if (flag) {
				if (ButtonMatrix == 2) {state = 2;} //4
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 1;
			}
			break;
		case 2:
			number = 4;
			if (flag) {
				if (ButtonMatrix == 1024) {state = 3;} //3
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 2;
			}
			break;
		case 3:
			number = 3;
			if (flag) {
				if (ButtonMatrix == 2) {state = 4;} //4
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 3;
			}
			break;
		case 4:
			number = 4;
			if (flag) {
				if (ButtonMatrix == 8) {state = 5;} //0
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 4;
			}
			break;
		case 5:
			number = 0;
			if (flag) {
				if (ButtonMatrix == 32) {state = 6;} //5
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 5;
			}
			break;
		case 6:
			number = 5;
			if (flag) {
				if (ButtonMatrix == 8) {state = 7;} //0
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 6;
			}
			break;
		case 7:
			number = 0;
			if (flag) {
				if (ButtonMatrix == 8) {state = 8;} //0
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 7;
			}
			break;
		case 8:
			number = 0;
			if (flag) {
				if (ButtonMatrix == 8) {state = 9;} //0
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 8;
			}
			break;
		case 9:
			number = 0;
			if (flag) {
				if (ButtonMatrix == 1024) {state = 10;} //3
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 9;
			}
			break;
		case 10:
			number = 3;
			if (flag) {
				if (ButtonMatrix == 8) {state = 11;} //0
				else if (ButtonMatrix == 4096) {state = 0;} //clear
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {state = 12;}
				state_L = 10;
			}
			break;
		case 11: //last number
			number = 0;
			if (flag) {
				if (ButtonMatrix == 32768) { //ok
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1); //LED1 on
					state = 11;
				}
				else if (ButtonMatrix == 4096) { //clear
					state = 0;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LED1 off
				}
				else if (ButtonMatrix == 8192){state -= 1;} //backspace
				else if (ButtonMatrix != 0 && ButtonMatrix != 32768 && ButtonMatrix != 128 && ButtonMatrix != 2048 && ButtonMatrix != 16384) {
					state = 12;
				}
				state_L = 11;
			}
			break;
		case 12: //input wrong number
			number = 999;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1); //LED2 on
			if (flag) {
				if (ButtonMatrix == 4096) { //clear
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LED1 off
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //LED2 off
					state = 0;
				}
				else if (ButtonMatrix == 8192){ //backspace
					state = state_L; //back to previous state
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //LED2 off
				}
			}
			break;
		}
		static uint32_t timestamp = 0;
		if (HAL_GetTick() >= timestamp) {
			timestamp = HAL_GetTick() + 100;
			ReadMatrixButton_1Row();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_8|LED_incorrect_number_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_incorrect_number_Pin */
  GPIO_InitStruct.Pin = LED_incorrect_number_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_incorrect_number_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ReadMatrixButton_1Row() {
	static uint8_t X = 0;
	register int i;
	for (i = 0; i < 4; i++) {
		if (HAL_GPIO_ReadPin(L[i].PORT, L[i].PIN)) {
			ButtonMatrix &= ~(1 << (X * 4 + i));
		} else {
			ButtonMatrix |= 1 << (X * 4 + i);
		}
	}
	HAL_GPIO_WritePin(R[X].PORT, R[X].PIN, 1);
	HAL_GPIO_WritePin(R[(X + 1) % 4].PORT, R[(X + 1) % 4].PIN, 0);
	X++;
	X %= 4;
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
