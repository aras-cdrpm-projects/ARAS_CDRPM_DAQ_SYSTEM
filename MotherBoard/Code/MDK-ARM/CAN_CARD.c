#include "main.h"
#include "string.h"

#define STATUS_ADDRESS		0
#define FORCE_ADDRESS			4
#define ENCODER_ADDRESS		8
#define DAC_ADDRESS				12
#define SPI_LINK_STATUS		16

#define RPU_CONVERT_FAIL_STATUS 1
#define RPU_CONVERT_SUCCESS_STATUS 0

//#define __CRUDE_REGS_SPI_SYS_DEBUG//Enable flag for crude register monitoring 
//#define 	__SENSOR_VAL_DEBUG				//Enable the debug flag for sensor readings
//#define 	__SENSOR_STATUS_DEBUG

volatile int SENSOR_RX_COUNTER=0;
volatile int SENSOR_RX_FLAG=0;
extern SPI_HandleTypeDef hspi2;
extern union UDP_packet outPacket;
extern char str[32];
extern UART_HandleTypeDef huart5;

union RPU_Packet{
	struct
	{
		int32_t encoder;
		int32_t analog;
	}Data;
	uint8_t buff[8];
}sensor_data[4];
uint8_t spi_rxBuffer[4][32] , spi_txBuffer[4][32] , spi_rxBuffer_shadow[4][32];
int 		CCU_RXFLAG[4]={0,0,0,0};

void initiate_transaction(int unit_number)
{
	spi_txBuffer[unit_number][31]=0x55; //Set the vertification byte
			switch(unit_number){
			case 0:
				HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,spi_txBuffer[unit_number],spi_rxBuffer_shadow[unit_number],32,100);
				if(spi_rxBuffer_shadow[unit_number][31]==0x55) //The data would be valid if the header value is correct
					memcpy((uint8_t*)&spi_rxBuffer[unit_number],(uint8_t*)spi_rxBuffer_shadow,32);
				HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_SET);
				break;
			case 1:
				HAL_GPIO_WritePin(CS3_GPIO_Port,CS3_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,spi_txBuffer[unit_number],spi_rxBuffer_shadow[unit_number],32,100);
				if(spi_rxBuffer_shadow[unit_number][31]==0x55) //The data would be valid if the header value is correct
					memcpy((uint8_t*)&spi_rxBuffer[unit_number],(uint8_t*)spi_rxBuffer_shadow[unit_number],32);
				HAL_GPIO_WritePin(CS3_GPIO_Port,CS3_Pin,GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(CS4_GPIO_Port,CS4_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,spi_txBuffer[unit_number],spi_rxBuffer_shadow[unit_number],32,100);
				if(spi_rxBuffer_shadow[unit_number][31]==0x55) //The data would be valid if the header value is correct
					memcpy((uint8_t*)&spi_rxBuffer[unit_number],(uint8_t*)spi_rxBuffer_shadow[unit_number],32);
				HAL_GPIO_WritePin(CS4_GPIO_Port,CS4_Pin,GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_RESET);
				HAL_SPI_TransmitReceive(&hspi2,spi_txBuffer[unit_number],spi_rxBuffer_shadow[unit_number],32,100);
				if(spi_rxBuffer_shadow[unit_number][31]==0x55) //The data would be valid if the header value is correct
					memcpy((uint8_t*)&spi_rxBuffer[unit_number],(uint8_t*)spi_rxBuffer_shadow[unit_number],32);
				HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_SET);
				#ifdef __CRUDE_REGS_SPI_SYS_DEBUG
						sprintf(str,"The Value is: ");
						HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
						for(int i=0;i<32;i++){
						sprintf(str,",%d",(int)spi_rxBuffer[unit_number][i]);
						HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
						}
						sprintf(str,"\r\n");
						HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
				#endif
				break;
		}
}

void CAN_CARDS_INIT(void){
	HAL_GPIO_WritePin(CS1_GPIO_Port,CS1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS2_GPIO_Port,CS2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS3_GPIO_Port,CS3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS4_GPIO_Port,CS4_Pin,GPIO_PIN_SET);
}


void spi_set_register(int address,int32_t val,int unit_number)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	packet.val=val;
	memcpy((uint8_t*)&spi_txBuffer[unit_number][address],(uint8_t*)packet.buff,4);
}

int32_t spi_get_register(uint32_t address,int unit_number)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	memcpy((uint8_t*)packet.buff,(uint8_t*)&spi_rxBuffer[unit_number][address],4);
	return packet.val;
}
void dacSend(int32_t dac[4])
{
	for(int i=0; i<4; i++){
		spi_set_register(DAC_ADDRESS,dac[i],i);
		initiate_transaction(i);
	}
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC2_Pin, GPIO_PIN_RESET);
}

void CC_Interrupt_rutine(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin){
		case CC1_IRQ_Pin:
				initiate_transaction(0);
				break;
		case CC2_IRQ_Pin:
				initiate_transaction(1);
				break;
		case CC3_IRQ_Pin:
				initiate_transaction(2);
				break;
		case CC4_IRQ_Pin:
				initiate_transaction(3);
				break;
	}
}

void sensorStartConversion(void)
{
	SENSOR_RX_COUNTER=0;
	memset((int *)CCU_RXFLAG,0,4);
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, CAN_SYNC1_Pin, GPIO_PIN_RESET);
}

void rpuGetSensorData(int32_t *encoder,int32_t *force,int channel)
{
	*force=		spi_get_register(FORCE_ADDRESS,channel);
	*encoder=	spi_get_register(ENCODER_ADDRESS,channel);
	#ifdef __SENSOR_VAL_DEBUG
		sprintf(str,"RPU %d readings: Force=%d , Encoder=%d\r\n",channel,*force,*encoder);
		HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
	#endif
}
void ccuHeartBeat(void)
{
	static uint16_t cnt=0;
	cnt++;
	if(cnt>200)
	{
		cnt=0;
		spi_set_register(SPI_LINK_STATUS,0x5555,0);
		spi_set_register(SPI_LINK_STATUS,0x5555,1);
		spi_set_register(SPI_LINK_STATUS,0x5555,2);
		spi_set_register(SPI_LINK_STATUS,0x5555,3);
	}
}

uint8_t getConversionStatus(void)
{
	uint8_t status=0;
	for(int i=0;i<4;i++)
		if(spi_get_register(STATUS_ADDRESS,i)==RPU_CONVERT_SUCCESS_STATUS)
			status|=(1<<i);
	#ifdef __SENSOR_STATUS_DEBUG
		sprintf(str,"RPUs Status are =%x \r\n",status);
		HAL_UART_Transmit(&huart5,(uint8_t *)str,strlen(str),100);
	#endif
	return status;
}

