#include "main.h"
#include "string.h"

#define STATUS_ADDRESS 0
#define RPU_SENSOR_ADDRESS 1
#define DAC_ADDRESS 9
#define RPU_CONVERT_FAIL_STATUS 1
#define RPU_CONVERT_SUCCESS_STATUS 2

volatile int SENSOR_RX_COUNTER=0;
volatile int SENSOR_RX_FLAG=0;
extern SPI_HandleTypeDef hspi2;

union{
	struct
	{
		int32_t encoder;
		int32_t analog;
	}Data;
	uint8_t buff[8];
}sensor_data[4];



void CAN_CARDS_INIT(void){
	HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS3_GPIO_Port,CS3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS4_GPIO_Port,CS4_Pin,GPIO_PIN_SET);
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

void CC_Interrupt_rutine(uint16_t GPIO_Pin)
{
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