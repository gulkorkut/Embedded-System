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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	GPIO_PinState btn_control_value = GPIO_PIN_RESET;
	GPIO_PinState btn_reset_value = GPIO_PIN_RESET;
	
	GPIO_PinState control_prev = GPIO_PIN_RESET;
	GPIO_PinState reset_prev = GPIO_PIN_RESET;
 
  uint16_t counter = 0;
	
	int i;
	
  while (1)
  {
		
		control_prev = btn_control_value;
		reset_prev = btn_reset_value;

		// read button value 
		btn_control_value = HAL_GPIO_ReadPin(btn_control_GPIO_Port, btn_control_Pin);
		btn_reset_value = HAL_GPIO_ReadPin(btn_reset_GPIO_Port, btn_reset_Pin);
		
		if(btn_reset_value != reset_prev){
			btn_reset_value = reset_prev == GPIO_PIN_SET? GPIO_PIN_RESET : GPIO_PIN_SET;
			
			if(btn_reset_value == GPIO_PIN_SET)
			{
				btn_control_value = GPIO_PIN_RESET;
			}
		}
		
		if(btn_control_value != control_prev){
			btn_control_value = control_prev == GPIO_PIN_SET? GPIO_PIN_RESET : GPIO_PIN_SET;
			
			if(btn_control_value == GPIO_PIN_SET)
			{
				btn_reset_value = GPIO_PIN_RESET;
			}
		}
		
		if(btn_reset_value == GPIO_PIN_SET){
			
			
			HAL_GPIO_WritePin(btn_control_GPIO_Port, btn_control_Pin, GPIO_PIN_RESET);
			counter = 0;
			HAL_GPIO_WritePin(bit_0_GPIO_Port, bit_0_Pin, GPIO_PIN_RESET);		
			HAL_GPIO_WritePin(bit_1_GPIO_Port, bit_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(bit_2_GPIO_Port, bit_2_Pin, GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(bit_3_GPIO_Port, bit_3_Pin, GPIO_PIN_RESET);
		}
		
		
		// check button condition
		if(btn_control_value == GPIO_PIN_SET) {
			
			// increase the num_press
			
			HAL_Delay(1000);
			counter++;
			/*
			for(i=0;i<0x100000;i++){
			}
			*/
			
			// get bit 0 and bit 1 value from current num_press
			GPIO_PinState bit0_value, bit1_value, bit2_value, bit3_value;
			bit0_value = (counter & (uint16_t)1) ? GPIO_PIN_SET:GPIO_PIN_RESET;
			bit1_value = (counter & (uint16_t)2) ? GPIO_PIN_SET:GPIO_PIN_RESET;
			bit2_value = (counter & (uint16_t)4) ? GPIO_PIN_SET:GPIO_PIN_RESET;
			bit3_value = (counter & (uint16_t)8) ? GPIO_PIN_SET:GPIO_PIN_RESET;

			// Turn LED on/off based on button value
			HAL_GPIO_WritePin(bit_0_GPIO_Port, bit_0_Pin, bit0_value);		
			HAL_GPIO_WritePin(bit_1_GPIO_Port, bit_1_Pin, bit1_value);
			HAL_GPIO_WritePin(bit_2_GPIO_Port, bit_2_Pin, bit2_value);	
			HAL_GPIO_WritePin(bit_3_GPIO_Port, bit_3_Pin, bit3_value);
		}
    

     		
		if(counter > 15) counter = 0;
		
		
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, bit_0_Pin|bit_1_Pin|bit_2_Pin|bit_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : btn_control_Pin btn_reset_Pin */
  GPIO_InitStruct.Pin = btn_control_Pin|btn_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : bit_0_Pin bit_1_Pin bit_2_Pin bit_3_Pin */
  GPIO_InitStruct.Pin = bit_0_Pin|bit_1_Pin|bit_2_Pin|bit_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
