#include "stm32f1xx_hal.h"
#include "string.h"

uint8_t spi_rxBuff[32],spi_txBuff[32];

uint8_t spi_shadow_in[64] ,spi_shadow_out[64];
uint8_t spi_test_out[64];



volatile uint8_t spi_regs[127];
volatile int spi_flag=0;
volatile uint8_t spi_rx=0;
volatile int spiByteCounter=0;
volatile uint8_t R_W=0;
extern volatile int sync1_intflag;


void spi_stack_init(SPI_HandleTypeDef *spi){
	memset((uint8_t*)spi_rxBuff,0x00,sizeof(spi_rxBuff));
	memset((uint8_t*)spi_txBuff,0x00,sizeof(spi_rxBuff));
	memset((uint8_t*)spi_rxBuff,0x00,sizeof(spi_shadow_out));
	memset((uint8_t*)spi_txBuff,0x00,sizeof(spi_shadow_in));
	spi_shadow_out[31]=0x55;// the vertification header 
	spi->Instance->CR1|=(1<<8); // Put the deevice in slave mode

}

void spi_stack_CS_Mng(SPI_HandleTypeDef *spi){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) //If the interrupt is generated due to a rising edge, The SPI module and the software SS signals
																					 //Have to be set
		{
			SPI1->CR1|=(1<<8); //SET the SW NSS Signal
			SPI1->CR1&=~(1<<6);//Desable The SPI Module so that the Buffers get flused
			//rearm the DMA and SPI System
			HAL_SPI_DMAStop(spi);
			if(spi_shadow_in[31]==0x55) // The received data is valid if the last byte is 0x55
				memcpy((uint8_t*)spi_rxBuff,spi_shadow_in,32);//copy the shadow buffer to the rx buffer
			HAL_SPI_TransmitReceive_DMA(spi,spi_shadow_out,spi_shadow_in,32); //Start The DMA in circular Mode and assign the buffers
		}else{
			SPI1->CR1&=~(1<<8); //Restet the SW NSS Signal
		}
}


void spi_dma_rx_complete_rutine(void){
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)==0) //The data in the buffer is for you if your SS is asserted
		memcpy((uint8_t*)spi_rxBuff,spi_shadow_in,32);//copy the shadow buffer to the rx buffer
}
void spi_out_shadow_update(void)
{
			memcpy((uint8_t*)spi_shadow_out,spi_txBuff,31);//copy the shadow buffer to the rx buffer
			spi_shadow_out[31]=0x55;// the vertification header 
}
void spi_set_register(int address,int32_t val)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	packet.val=val;
	memcpy((uint8_t*)&spi_txBuff[address],(uint8_t*)packet.buff,4);
}

int32_t spi_get_register(int address)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	memcpy((uint8_t*)packet.buff,(uint8_t*)&spi_rxBuff[address],4);
	return packet.val;
}
void spi_set_rx_chache_register(int address,int32_t val)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	packet.val=val;
	memcpy((uint8_t*)&spi_rxBuff[address],(uint8_t*)packet.buff,4);
}

