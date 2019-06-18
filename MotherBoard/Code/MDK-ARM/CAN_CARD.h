#ifndef __CAN_CARD
#define __CAN_CARD
extern volatile int SENSOR_RX_COUNTER;
extern volatile int SENSOR_RX_FLAG;
extern union{
	struct
	{
		int32_t encoder;
		int32_t analog;
	}Data;
	uint8_t buff[8];
}sensor_data[4];

void CAN_CARDS_INIT(void);
void CCU_Write(uint8_t Address,uint8_t *data,int len , int CCU_CS);
void dacSend(int32_t dac[4]);
void CCU_Read(uint8_t Address,uint8_t *data,int len , int CCU_CS);
void CC_Interrupt_rutine(uint16_t GPIO_Pin);
void sensorStartConversion(void);
void ccuHeartBeat(void);
void rpuGetSensorData(int32_t *encoder,int32_t *force,int channel);
uint8_t getConversionStatus(void);


#endif

