#include "main.h"
#include "string.h"
#include "CAN_CARD.h"

#define ACCELEROMETER_ID 0x01
#define GYROSCOPE_ID		 0X02
#define MAGNETOMETER_ID	 0X03
#define BAROMETER_ID		 0x04
#define CAM_POS_ID			 0x05

//#define CAN_DEBUG_ENABLE
extern UART_HandleTypeDef huart5;
extern char str[32];
volatile int IMU_RX_FLAG=0;

union{
	struct{
		int16_t a;
		int16_t b;
		int16_t c;
	}Data;
	uint8_t buffer[6];
}IMU_Mess;
union{
	struct{
		int32_t barometer;
	}Data;
	uint8_t buffer[4];
}Barometer_Mess;
int16_t accel[3];
int16_t gyro[3];
int16_t magnetometer[3];
int32_t barometer;

	
CAN_RxHeaderTypeDef TelRMess;
CAN_TxHeaderTypeDef TelTMess;
uint8_t TelTxData[8] = {0};
uint32_t TelTxMailBox;

void can_stack_init(CAN_HandleTypeDef *hcan)
{
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
	
	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	if (HAL_CAN_Start(hcan) != HAL_OK)
  {
    Error_Handler();
  }

	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void can_stack_interrupt_rutine(CAN_HandleTypeDef *hcan)
{
	uint8_t canData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &TelRMess, canData) != HAL_OK){Error_Handler();}
	switch(TelRMess.StdId)
	{
		case ACCELEROMETER_ID:
			IMU_Tick();
			sensorStartConversion(); //Initiate the conversion as soon as the first portion of IMU packet is received
			memcpy((uint8_t*)IMU_Mess.buffer,canData,TelRMess.DLC);
			accel[0]=IMU_Mess.Data.a;
			accel[1]=IMU_Mess.Data.b;
			accel[2]=IMU_Mess.Data.c;
			#ifdef CAN_DEBUG_ENABLE
				sprintf(str,"RPUs Status are =%x \r\n",accel[0]);
				HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
			#endif
			break;
		case GYROSCOPE_ID:
			memcpy((uint8_t*)IMU_Mess.buffer,canData,TelRMess.DLC);
			gyro[0]=IMU_Mess.Data.a;
			gyro[1]=IMU_Mess.Data.b;
			gyro[2]=IMU_Mess.Data.c;
			break;
		case MAGNETOMETER_ID:
			memcpy((uint8_t*)IMU_Mess.buffer,canData,TelRMess.DLC);
			magnetometer[0]=IMU_Mess.Data.a;
			magnetometer[1]=IMU_Mess.Data.b;
			magnetometer[2]=IMU_Mess.Data.c;
		  IMU_RX_FLAG=1; // Let the main loop know that the data is ready to be transmitted
			break;
		case BAROMETER_ID:
			memcpy((uint8_t*)Barometer_Mess.buffer,canData,TelRMess.DLC);
			barometer=Barometer_Mess.Data.barometer;
			break;
	}
}