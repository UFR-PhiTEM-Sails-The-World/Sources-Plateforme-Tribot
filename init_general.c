#include <stm32f4xx_hal.h>
#include "init_general.h"

UART_HandleTypeDef huart2;

void init_TIM5_base_500us(void);

void MX_USART2_UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	// Peripheral clock enable 
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	//USART2 GPIO Configuration    
	//PA2     ------> USART2_TX
	//PA3     ------> USART2_RX 
	
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
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
    //Error_Handler();
  }

}


 void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  //Configure the main internal regulator output voltage 
  
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  // Initializes the CPU, AHB and APB busses clocks 
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
  }
  // Activate the Over-Drive mode 
  
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    //Error_Handler();
  }
  // Initializes the CPU, AHB and APB busses clocks 
  
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    //Error_Handler();
  }
}


int fputc(int ch, FILE *f){
	 HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
		//return ITM_SendChar(ch);

return ch;
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}




// 
void init_general(void){
	HAL_Init();
	SystemClock_Config();
	init_TIM5_base_500us();
	MX_USART2_UART_Init();

    printf("Initialisation finie !!!!!\n\r");
		
}
void init_TIM5_base_500us(void){
	RCC->APB1ENR |= (1<<3);
	TIM5->CR1 = (1<<3);
	TIM5->PSC = 44999;// T_CK_CNT = 500us 
	//TIM5->ARR = 99;
	TIM5->EGR |= 1;
}
void tempo_TIM5_x_1ms(unsigned int x){
		TIM5->ARR = 2*x;
		TIM5->CR1 |= 1;
		while( (TIM5->CR1&1) == 1);	
}

