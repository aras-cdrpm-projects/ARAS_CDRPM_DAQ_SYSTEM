/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "spi_stack.h"

#define CONVERT_FAIL_STATUS 1
#define CONVERT_SUCCESS_STATUS 0

#define SPI_SYS_DBUG_MODE 0	//Set this flag to force the CAN subsytem into a simulated data exchange mode. This way we can debug the spi system in an isolated manner
//The artificial values for the sensors in the SPI debug mode
#define ENCODER_DBG_VAL   7
#define FORCE_DBG_VAL			8
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
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//********************************CAN******************************
	CAN_RxHeaderTypeDef RMess;
	CAN_TxHeaderTypeDef TMess;
	uint8_t TxData[8] = {0};
 	uint8_t canRxData[8];
	uint32_t TxMailBox;
	HAL_StatusTypeDef canret;
	volatile int RX_FLAG=0, state=0;
	volatile int RPU_CAN_READ_SENSORS_REQ=0;
	int32_t CAN_CNVT_TIME;
	int LED_BeatFlag=0;
	int CAN_LINK_STATUS=CONVERT_FAIL_STATUS; 
char message[32];
	union{
	struct{
	int16_t angOut;
	}Data;
	uint8_t buffer[2];
}CAN_OUT_ANG_PACK;

union{
	struct{
	int32_t enc;
	int16_t force;
	}Data;
	uint8_t buffer[6];
}CAN_IN_SIG_PACK;

//*****************************************************************
char str[100];
extern volatile int spi_tx_finish_flag, sync2_ready;
extern int LED_OFF_DELAY;
extern int LED_ON_DELAY;
extern int LED_WD;
volatile int convert_done_flag=0,sync1_intflag=0;
int32_t convert_time;
//when the Value of the sensor data is received form the RPU unit, its value is stored using this function
void reg_load_converted(void)
{
  int32_t encoder;
	int32_t force;

	if(SPI_SYS_DBUG_MODE)
	{
		encoder=ENCODER_DBG_VAL;
		force	=FORCE_DBG_VAL;
	}
	else
	{
		encoder=CAN_IN_SIG_PACK.Data.enc;
		force	=CAN_IN_SIG_PACK.Data.force;
	}
	//sprintf(message,"Force Measurement= %d \r\n",CAN_IN_SIG_PACK.Data.force);
	//HAL_UART_Transmit(&huart1,message,sizeof(message),10);
	spi_set_register(FORCE_ADDRESS,force);
	spi_set_register(ENCODER_ADDRESS,encoder);
}

//Based on wether the data transaction to the RPU has been valid or not, this function sets the status register
void setStatusReg(int status)
{
	spi_set_register(STATUS_ADDRESS,status);
	CAN_LINK_STATUS=status;
}

//This state maching handles the Motherboard-Card connection which is realized through SPI connection
void SPIStateMachine(void)
{
	static int state=0;
	switch(state)
	{
		case 0:
			if(sync1_intflag) // Start the process when an Interrupt is detected on the SYNC1 Pin
			{
				sync1_intflag=0;
				convert_time=HAL_GetTick();
				RPU_CAN_READ_SENSORS_REQ=1;//Trigger the CAN state machine to initiate a conversion and get the results.
				state=1;
			}else
			{
				state=0;
				convert_done_flag=0;
			}
			break;
			
		case 1:
			if(convert_done_flag==1)
			{
				convert_done_flag=0;
				reg_load_converted();
				setStatusReg(CONVERT_SUCCESS_STATUS);
				spi_out_shadow_update();
				HAL_GPIO_WritePin(IRQ_Req_GPIO_Port,IRQ_Req_Pin,GPIO_PIN_SET);
				state=4;
			}else
			{
				state=2;
			}
			break;
		case 2:
			if((HAL_GetTick()-convert_time)<100)
			{
				state=1;
			}
			else
			{
				setStatusReg(CONVERT_FAIL_STATUS);
				spi_out_shadow_update();
				HAL_GPIO_WritePin(IRQ_Req_GPIO_Port,IRQ_Req_Pin,GPIO_PIN_SET);
				state=3;
			}
			break;
		case 3:
			HAL_GPIO_WritePin(IRQ_Req_GPIO_Port,IRQ_Req_Pin,GPIO_PIN_RESET);
			state=0;
			break;
		case 4:
			HAL_GPIO_WritePin(IRQ_Req_GPIO_Port,IRQ_Req_Pin,GPIO_PIN_RESET);
			state=0;
			break;
		default:
			state=0;
	}

}
//When there is a read sensor request, this state machine handles reading them from RPU
void CAN_GET_SENSORS_SM(void)
{
	static int state=0;
	if(RPU_CAN_READ_SENSORS_REQ==1)
		{
			switch(state)
			{
				case 0:
					RX_FLAG=0;
					TMess.DLC=0x0;
					TMess.StdId=0x1;
					if(!SPI_SYS_DBUG_MODE)
						if(HAL_CAN_AddTxMessage(&hcan, &TMess, 0, &TxMailBox)!=HAL_OK){Error_Handler();} //Send the convert CMD
					CAN_CNVT_TIME=HAL_GetTick();
					state=1;
					break;
			
				case 1:
					if(SPI_SYS_DBUG_MODE)
						RX_FLAG=1;
					if(RX_FLAG==1){
						//Process The data Received from the Slave device
						RX_FLAG=0;
						convert_done_flag=1; //Tell the SPI supsystem that the conversion is done
						state=2;
					}
					else{
						state=3;
					}
					break;
					
				case 3:
					if((HAL_GetTick()-CAN_CNVT_TIME)<20)
					{
						state=1;
					}else
					{
						//No responce is received from the slave device
						RPU_CAN_READ_SENSORS_REQ=0;
						state=0;
					}
					break;
				
				case 2:/*
						//Write to the Analog Output of the slave device
						TMess.DLC=0x2;
						TMess.StdId=0x2;
						CAN_OUT_ANG_PACK.Data.angOut=50;
						if(HAL_CAN_AddTxMessage(&hcan, &TMess,CAN_OUT_ANG_PACK.buffer, &TxMailBox)!=HAL_OK){Error_Handler();}
						*/
						state=0;
						RPU_CAN_READ_SENSORS_REQ=0;
						break;
				default:
					state=0;
		}
	}
}
//this state machine handles sending the DAC setpoint of the RPUs through CAN BUS
void send_loaded_packs_SM(void)
{
	if(sync2_ready==1){
		sync2_ready=0;
		union
		{
			int32_t dac;
			uint8_t buff[4];
		}Outpack;
		Outpack.dac=spi_get_register(12);
		TMess.DLC=0x2;
		TMess.StdId=0x2;
		if(HAL_CAN_AddTxMessage(&hcan, &TMess,Outpack.buff, &TxMailBox)!=HAL_OK){Error_Handler();}
		}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	//**************************CAN************************************
	TMess.RTR = CAN_RTR_DATA;
  TMess.IDE = CAN_ID_STD;
  TMess.DLC = 0x2;
	TMess.StdId=0x1;
	TMess.TransmitGlobalTime = DISABLE;
	

	//*****************************************************************
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
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	spi_stack_init(&hspi1);
	HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		if(spi_get_register(SPI_LINK_STATUS)==0x5555)
			{
				LED_WD=0;
				spi_set_rx_chache_register(SPI_LINK_STATUS,0);
				if(CAN_LINK_STATUS==CONVERT_FAIL_STATUS)
				{
					LED_ON_DELAY=200;
					LED_OFF_DELAY=200;
				}
				else
				{
					LED_ON_DELAY=100;
					LED_OFF_DELAY=1200;
				}
		  }
		
     SPIStateMachine();
		 CAN_GET_SENSORS_SM();
		 send_loaded_packs_SM();
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */
	CAN_FilterTypeDef  sFilterConfig;
	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 12;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 9;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IRQ_Req_GPIO_Port, IRQ_Req_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : sync1_intflag_Pin PB1 */
  GPIO_InitStruct.Pin = sync1_intflag_Pin|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Req_Pin */
  GPIO_InitStruct.Pin = IRQ_Req_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IRQ_Req_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RMess, canRxData) != HAL_OK)
  {
    Error_Handler();
  }
	
	if((RMess.StdId==0x1)&&(RMess.IDE==CAN_ID_STD)){
		RX_FLAG=1;
		memcpy(CAN_IN_SIG_PACK.buffer,canRxData,6);
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
