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
struct PortPin{
	GPIO_TypeDef* PORT;
	uint16_t PIN;
};

struct PortPin R[4] = {
		{GPIOA,GPIO_PIN_10}, //A10
		{GPIOC,GPIO_PIN_9}, //C9
		{GPIOB,GPIO_PIN_5}, //B5
		{GPIOB,GPIO_PIN_8} //B8
};

struct PortPin L[4] = {
		{GPIOA,GPIO_PIN_9}, //A9
		{GPIOC,GPIO_PIN_7}, //C7
		{GPIOB,GPIO_PIN_6}, //B6
		{GPIOA,GPIO_PIN_7} //A7
};

uint16_t ButtonMatrix = 0;
int state = 0;

int num[] = {8,
		  	4, 64, 1024,
			2, 32, 512,
			1, 16, 256};
int cmd[] = {32768, 4096}; //ok, clear
int empty[] = {128, 2048, 16384}; //empty switch
int bs[] = {8192};

int num_sw = 0;

int i = 0;

int m = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ReadMatrixButton_1Row();
int chkIncNum(uint16_t incorrect_num, int correct_num);
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
  while (1){
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //state transition
	  switch (state) {
		case 0:
			//initial LED off
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0); //LED for ok
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0); //LED for state 12
			num_sw = 6;
			if (ButtonMatrix == num[num_sw]) {state = 1;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			break;
		case 1:
			num_sw = 4;
			if (ButtonMatrix == num[num_sw]) {state = 2;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw) == 9){
				m = 12;
				state = 12;
			}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12; i = 1;};
			break;
		case 2:
			num_sw = 3;
			if (ButtonMatrix == num[num_sw]) {state = 3;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 3:
			num_sw = 4;
			if (ButtonMatrix == num[num_sw]) {state = 4;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 4:
			num_sw = 0;
			if (ButtonMatrix == num[num_sw]) {state = 5;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 5:
			num_sw = 5;
			if (ButtonMatrix == num[num_sw]) {state = 6;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 6:
			num_sw = 0;
			if (ButtonMatrix == num[num_sw]) {state = 7;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 7:
			num_sw = 0;
			if (ButtonMatrix == num[num_sw]) {state = 8;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 8:
			num_sw = 0;
			if (ButtonMatrix == num[num_sw]) {state = 9;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 9:
			num_sw = 3;
			if (ButtonMatrix == num[num_sw]) {state = 10;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 10:
			num_sw = 0;
			if (ButtonMatrix == num[num_sw]) {state = 11;}
			else if (ButtonMatrix == cmd[1]){state = 0;}
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != cmd[0] && ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;};
			break;
		case 11: //last state
			num_sw = 99;
			if (ButtonMatrix == cmd[0]) {
				state = 11;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			} //ok -> LED on
			else if (ButtonMatrix == cmd[1]){
				state = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
			} //clear -> LED off
			else if (chkIncNum(ButtonMatrix, num_sw)){state = 12;}
//			else if (ButtonMatrix != empty[0] && ButtonMatrix != empty[1] && ButtonMatrix != empty[2]){state = 12;}
			break;
		case 12: //case: input wrong number
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
			if (ButtonMatrix == cmd[1]) {state = 0;} //clear
			break;
	  }
	  static uint32_t timestamp = 0;
	  	  if(HAL_GetTick()>= timestamp){
	  		  timestamp = HAL_GetTick()+ 100;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);

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
int chkIncNum(uint16_t incorrect_num, int correct_num){
	for(int k=0; k<10; k++){
		if(correct_num != num[k]){ //skip correct number
			if (incorrect_num == num[k]) { //check if incorrect number in num[]
				return 9;
			}
		}
	}
	return 7;
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
