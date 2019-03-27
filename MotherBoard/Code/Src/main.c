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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "udp.h"
#include "spi_stack.h"
#define STATUS_ADDRESS 0
#define RPU_SENSOR_ADDRESS 1
#define DAC_ADDRESS 9

#define RPU_CONVERT_FAIL_STATUS 1
#define RPU_CONVERT_SUCCESS_STATUS 2
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
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_GFXSIMULATOR_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
struct udp_pcb *UDP;
ip_addr_t Remote_IP, Local_IP;
extern volatile int RPI_SYNC;
extern volatile int UDP_TFLAG;
struct pbuf *Transmit_Pbuf;
volatile int SENSOR_RX_COUNTER=0;

/*
union{
	struct
	{
		struct
		{
			int32_t encoder;
			int32_t analog;
		}sensor_data[4];
		struct
		{
			int16_t x;
			int16_t y;
			int16_t z;
		}IMU[2];
		float pos_x;
		float pos_y;
		float pos_z;
	}Data;
	uint8_t buff[56];
}outPacket;
*/
union{
	struct
	{
		int32_t a;
		int32_t b;
	}Data;
	uint8_t buff[8];
}outPacket;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_RxHeaderTypeDef RMess;
CAN_TxHeaderTypeDef TMess;
uint8_t TxData[8] = {0};
uint8_t canRxData[8];
uint32_t TxMailBox;
HAL_StatusTypeDef canret;

volatile char str[32];
volatile int SENSOtgRX_COUNTER=0;
volatile int SENSOR_RX_FLAG=0;
extern volatile int nob_encoder_index;
int32_t nob_encoder_read(void);

void CAN_CARDS_INIT(void){
	HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS3_GPIO_Port,CS3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS4_GPIO_Port,CS4_Pin,GPIO_PIN_SET);
}
union{
	struct
	{
		int32_t encoder;
		int32_t analog;
	}Data;
	uint8_t buff[8];
}sensor_data[4];

/////////////////RPI CODE STARTS HERE

void RPI_DATA_LOAD(void){

	//memcpy((uint8_t*)&regs[1],RPI_DATA.buff,8);
}

void setStatusReg(int status)
{
	//regs[STATUS_ADDRESS]=status;
}
/////////////////RPI CODE ENDS HERE

void sensorStartConversion(void)
{
	SENSOR_RX_COUNTER=0;
	SENSOR_RX_FLAG=0;
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC1_Pin, GPIO_PIN_RESET);
}

void CCU_Write(uint8_t Address,uint8_t *data,int len , int CCU_CS) //CAN CARD Unit Write Regs

{
	  uint8_t rxBuf[len+1];
	  uint8_t txBuf[len+1];
	  txBuf[0]=Address&0x7F;
		memcpy(&txBuf[1],data,len);
		switch(CCU_CS){
			case 1:
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOF,CS2_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS2_Pin,GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOF,CS3_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS3_Pin,GPIO_PIN_SET);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOF,CS4_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS4_Pin,GPIO_PIN_SET);
				break;
			default:
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_SET);
		}
		
}

void dacSend(int32_t dac[4])
{
	union{
	int32_t dac;
	uint8_t buff[4];
	}sensor_data;
	
	for(int i=0; i<4; i++){
		sensor_data.dac=dac[i];
		CCU_Write(9, sensor_data.buff, sizeof(sensor_data.buff), i);
	}
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC2_Pin, GPIO_PIN_RESET);
}

void CCU_Read(uint8_t Address,uint8_t *data,int len , int CCU_CS) ////CAN CARD Unit Read Regs
{
	  uint8_t txBuf[len+1];
		txBuf[0]=((uint8_t)(1<<7))|Address;
	  uint8_t rxBuf[len+1];
		switch(CCU_CS){
			case 1:
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100);  
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOF,CS2_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS2_Pin,GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOF,CS3_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS3_Pin,GPIO_PIN_SET);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOF,CS4_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100); 
				HAL_GPIO_WritePin(GPIOF,CS4_Pin,GPIO_PIN_SET);
				break;
			default:
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,txBuf,rxBuf,len+1,100);  
				HAL_GPIO_WritePin(GPIOE,CS1_Pin,GPIO_PIN_SET);
			}
		memcpy(data,&rxBuf[1],len);
}
CAN_TxHeaderTypeDef TMess;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	outPacket.Data.a=1;
	outPacket.Data.b=10;
	TMess.RTR = CAN_RTR_DATA;
  TMess.IDE = CAN_ID_STD;
  TMess.DLC = 0x2;
	TMess.StdId=0x1;
	
	float pos[]={1.5,2.5,3.5};
	float q[]={0,2,3,4};


	uint8_t keys[4];
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI2_Init();
  MX_UART5_Init();
  MX_LWIP_Init();
  MX_TIM1_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_GFXSIMULATOR_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim4);
	CAN_CARDS_INIT();
	HAL_GPIO_WritePin(GPIOD, RPi_PW_Pin, GPIO_PIN_RESET);
	nob_encoder_index=0;	//Reset the encoder index in case an unwanted interrupt occures.
	IP_ADDR4(&Local_IP, 192, 168, 50, 100);
	IP_ADDR4(&Remote_IP, 192, 168, 50, 110);
	UDP=udp_new();
	udp_bind(UDP, IP_ADDR_ANY, 5500);
	Transmit_Pbuf=pbuf_alloc(PBUF_TRANSPORT, sizeof(outPacket.Data), PBUF_RAM);
	spi_stack_init(&hspi1);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(500);
	uint8_t data[2]={1 , 2};
	spi_regs[0]=1;
	spi_regs[1]=2;
	spi_regs[2]=3;
	spi_regs[3]=4;
  while (1)
	{
		//HAL_CAN_AddTxMessage(&hcan1, &TMess, data, &TxMailBox);
		readKeys(keys);
		pos[0]=(float)keys[0];
		pos[1]=(float)keys[1];
		pos[2]=(float)keys[2];
		IMP_Write(pos,q);
		HAL_Delay(1);
		//
		/*
			sprintf(str,"\n\n%x ,\t %x\n\r",sensor_data[0].Data.analog,sensor_data[0].Data.encoder);
			HAL_UART_Transmit(&huart5, str, sizeof(str), 10);	
			sprintf(str,"%x ,\t %x\n\r",sensor_data[1].Data.analog,sensor_data[1].Data.encoder);
			HAL_UART_Transmit(&huart5, str, sizeof(str), 10);	
			sprintf(str,"%x ,\t %x\n\r",sensor_data[2].Data.analog,sensor_data[2].Data.encoder);
			HAL_UART_Transmit(&huart5, str, sizeof(str), 10);	
			sprintf(str,"%x ,\t %x\n\r",sensor_data[3].Data.analog,sensor_data[3].Data.encoder);
			HAL_UART_Transmit(&huart5, str, sizeof(str), 10);	
		*/
		int vals[4]={1000,2000,3000,4000};
		if (UDP_TFLAG==1){
			//dacSend(vals);
			//sensorStartConversion();
			//while(SENSOR_RX_FLAG==0);
			UDP_TFLAG=0;
			//pbuf_take(Transmit_Pbuf, outPacket.buff, sizeof(outPacket.buff));
			//udp_sendto(UDP, Transmit_Pbuf, &Remote_IP, 5500);
		}
		MX_LWIP_Process();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
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
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 18;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_2TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
	//if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  //{
  //  Error_Handler();
  //}
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GFXSIMULATOR Initialization Function
  * @param None
  * @retval None
  */
static void MX_GFXSIMULATOR_Init(void)
{

  /* USER CODE BEGIN GFXSIMULATOR_Init 0 */

  /* USER CODE END GFXSIMULATOR_Init 0 */

  /* USER CODE BEGIN GFXSIMULATOR_Init 1 */

  /* USER CODE END GFXSIMULATOR_Init 1 */
  /* USER CODE BEGIN GFXSIMULATOR_Init 2 */

  /* USER CODE END GFXSIMULATOR_Init 2 */

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
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 143;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 9999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS4_Pin|CS3_Pin|CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, TEST_Pin|CAN_SYNC1_Pin|CAN_SYNC2_Pin|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RPi_PW_GPIO_Port, RPi_PW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS4_Pin CS3_Pin CS2_Pin */
  GPIO_InitStruct.Pin = CS4_Pin|CS3_Pin|CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TEST_Pin CAN_SYNC1_Pin CAN_SYNC2_Pin USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = TEST_Pin|CAN_SYNC1_Pin|CAN_SYNC2_Pin|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CC1_IRQ_Pin CC2_IRQ_Pin CC3_IRQ_Pin CC4_IRQ_Pin */
  GPIO_InitStruct.Pin = CC1_IRQ_Pin|CC2_IRQ_Pin|CC3_IRQ_Pin|CC4_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : RPi_PW_Pin */
  GPIO_InitStruct.Pin = RPi_PW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RPi_PW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	uint8_t status;
	switch(GPIO_Pin){
		case(CC1_IRQ_Pin):
			SENSOR_RX_COUNTER++;
			CCU_Read(STATUS_ADDRESS, &status, 1, 1);
			if(status==RPU_CONVERT_SUCCESS_STATUS){
				CCU_Read(RPU_SENSOR_ADDRESS,sensor_data[0].buff, 8, 1);
			}
			else{
				sensor_data[0].Data.analog=-1;
				sensor_data[0].Data.encoder=-1;
			}
			break;
		case(CC2_IRQ_Pin):
			SENSOR_RX_COUNTER++;
			CCU_Read(STATUS_ADDRESS, &status, 1, 1);
			if(status==RPU_CONVERT_SUCCESS_STATUS){
				CCU_Read(RPU_SENSOR_ADDRESS,sensor_data[1].buff, 8, 2);		
			}
			else{
				sensor_data[0].Data.analog=-1;
				sensor_data[0].Data.encoder=-1;			}
			break;
		case(CC3_IRQ_Pin):
			SENSOR_RX_COUNTER++;
			CCU_Read(STATUS_ADDRESS, &status, 1, 1);
			if(status==RPU_CONVERT_SUCCESS_STATUS){
				CCU_Read(RPU_SENSOR_ADDRESS,sensor_data[2].buff, 8, 3);
			}
			else{
				sensor_data[0].Data.analog=-1;
				sensor_data[0].Data.encoder=-1;			}
			break;
		case(CC4_IRQ_Pin):
			SENSOR_RX_COUNTER++;
			CCU_Read(STATUS_ADDRESS, &status, 1, 1);
			if(status==RPU_CONVERT_SUCCESS_STATUS){
				CCU_Read(RPU_SENSOR_ADDRESS,sensor_data[3].buff, 8, 4);
				}
			else{
				sensor_data[0].Data.analog=-1;
				sensor_data[0].Data.encoder=-1;			}
			break;
		case(GPIO_PIN_4): //RPI_CS
		break;
	}
	
	if(SENSOR_RX_COUNTER==3)
		SENSOR_RX_FLAG=1;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RMess, canRxData) != HAL_OK)
  {
    Error_Handler();
  }
	
	if((RMess.StdId==(uint8_t)10)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(IMP.buffer, canRxData, 8);
	}
	if((RMess.StdId==(uint8_t)11)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMP.buffer[9], canRxData, 4);
	}
	if((RMess.StdId==(uint8_t)12)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMP.buffer[13], canRxData, 8);
	}
	if((RMess.StdId==(uint8_t)13)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMP.buffer[21], canRxData, 6);
	}
	if((RMess.StdId==(uint8_t)14)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMU.buffer, canRxData, 6);
	}
	if((RMess.StdId==(uint8_t)15)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMU.buffer[13], canRxData, 6);
	}
	if((RMess.StdId==(uint8_t)16)&&(RMess.IDE==CAN_ID_STD)){
		//memcpy(&IMU.buffer[25], canRxData, 6);
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
