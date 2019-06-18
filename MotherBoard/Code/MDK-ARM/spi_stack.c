#include "stm32f7xx_hal.h"
#include "string.h"
#define IMP_BASEADDRESS     0
#define IMU_BASEADDRESS     30
#define RPUs_BASEADDRESS    68
#define KEYS_BASEADDRESS    102
#define SLIDERS_BASEADDRESS 110
uint8_t spi_rxBuff[128],spi_txBuff[128];
uint8_t rxBuff[128],txBuff[128];

    union{
        struct
        {
            float Pos[3];
            float q[4];

        }Data;
        uint8_t buffer[28];
    }IMP;//28 Bytes
    union{
        struct
        {
            int32_t m[3];
            int32_t a[3];
            int32_t g[3];
        }Data;
        uint8_t buffer[36];
    }IMU;//36 Bytes
    union{
        struct
        {
            uint8_t Keys[4];
        }Data;
        uint8_t buffer[4];
    }KEYs;//4 Bytes
    union{
        struct
        {
            uint8_t sliders[4];
        }Data;
        uint8_t buffer[4];
    }Sliders;//4 Bytes
    union{
        struct
        {
            int32_t  Enc[4];
            int32_t  force[4];
        }Data;
        uint8_t buffer[32];
    }RPUs;//32 Bytes
volatile uint8_t spi_regs[127];
volatile int spi_flag=0;
volatile uint8_t spi_rx=0;
volatile int spiByteCounter=0, sync2_ready=0;
volatile uint8_t address;
volatile uint8_t R_W=0;
extern volatile int sync1_intflag;
void regRead(uint8_t Address, uint8_t *data, int len)
{
    memcpy((uint8_t *)data,(uint8_t *)&spi_rxBuff[Address],len);
}
void regWrite(uint8_t Address,uint8_t *data,int len)
{
    memcpy((uint8_t *)&spi_txBuff[Address],(uint8_t *)data,len);
}
void spi_stack_init(void){
	memset((uint8_t*)spi_rxBuff,0x00,sizeof(spi_rxBuff));
	memset((uint8_t*)spi_txBuff,0x00,sizeof(spi_rxBuff));
	//DMA Config, DMA2 Stream 0 is for SPI_RX and DMA2 Stream 3 is for SPI TX
	//DAM2 Stream0 SPI_RX
	DMA2_Stream0->CR=0;
	DMA2_Stream0->CR|=(3<<25);//set Chanel 3
	DMA2_Stream0->CR|=(2<<16);//Prrority
	DMA2_Stream0->CR|=(1<<10);//mem Address incriment
	DMA2_Stream0->CR|=(1<<4);//Transfer Complete interrupt enable
	//DAM2 Stream0 SPI_TX
	DMA2_Stream3->CR=0;
	DMA2_Stream3->CR|=(3<<25);//set Chanel 3
	DMA2_Stream3->CR|=(2<<16);//Prrority
	DMA2_Stream3->CR|=(1<<10);//mem Address incriment
	DMA2_Stream3->CR|=(1<<6);//Memory to pripheral transfer 
	//Set the DMA addresses
	DMA2_Stream0->PAR=(volatile unsigned int)&(SPI1->DR);
	DMA2_Stream0->M0AR=(volatile unsigned int)rxBuff;

	DMA2_Stream3->PAR=(unsigned int)&(SPI1->DR);
	DMA2_Stream3->M0AR=(unsigned int)txBuff;
}

void spi_stack_CS_Mng(void){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) //If the interrupt is generated due to a rising edge, The SPI module and the software SS signals
																					 //Have to be set
		{
			DMA2_Stream0->CR&=~(1<<0);
			DMA2_Stream3->CR&=~(1<<0);
			DMA2->LIFCR=0xffffffff; //Clear All DMA Flags
			SPI1->CR1|=(1<<8); //SET the SW NSS Signal
			SPI1->CR1&=~(1<<6);// Enable The SPI Module
			SPI1->CR2&=~(1<<0); //Disable the DMA FIFO in the SPI Module 
		  SPI1->CR2&=~(1<<1);
		}else{
			memcpy((uint8_t*)txBuff,(uint8_t *)spi_txBuff,sizeof(spi_txBuff));
			SPI1->CR2|=(1<<0);
		  SPI1->CR2|=(1<<1);
			DMA2_Stream0->NDTR=128;
		  DMA2_Stream3->NDTR=128;
			DMA2->LIFCR=0xffffffff; //Clear all Flags
			DMA2_Stream0->CR|=(1<<0);
			DMA2_Stream3->CR|=(1<<0);
			SPI1->CR1&=~(1<<8); //Restet the SW NSS Signal
			SPI1->CR1|=(1<<6);// Enable The SPI Module
		}
}


void spi_dma_rx_complete_rutine(void){
	memcpy((uint8_t*)spi_rxBuff,(uint8_t *)rxBuff,sizeof(rxBuff));
}
void IMP_Write(float *pos, float *q)
{
    IMP.Data.Pos[0]=pos[0];
    IMP.Data.Pos[1]=pos[1];
    IMP.Data.Pos[2]=pos[2];
    IMP.Data.q[0]=q[0];
    IMP.Data.q[1]=q[1];
    IMP.Data.q[2]=q[2];
    IMP.Data.q[3]=q[3];

		regWrite(IMP_BASEADDRESS,IMP.buffer,sizeof(IMP.buffer)); 
}

void IMU_Write(int32_t *a, int32_t *g, int32_t *m)
{
    IMU.Data.a[0]=a[0];
    IMU.Data.a[1]=a[1];
    IMU.Data.a[2]=a[2];

    IMU.Data.g[0]=g[0];
	  IMU.Data.g[1]=g[1];
    IMU.Data.g[2]=g[2];
	
    IMU.Data.m[0]=m[0];
		IMU.Data.m[1]=m[1];
    IMU.Data.m[2]=m[2];
   	regWrite(IMU_BASEADDRESS,IMU.buffer,sizeof(IMU.buffer)); 

}

void RPUs_Write(int32_t *encoders, int32_t *forces)
{
    RPUs.Data.Enc[0]=encoders[0];
    RPUs.Data.Enc[1]=encoders[1];
    RPUs.Data.Enc[2]=encoders[2];
    RPUs.Data.Enc[3]=encoders[3];

    RPUs.Data.force[0]=forces[0];
    RPUs.Data.force[1]=forces[1];
    RPUs.Data.force[2]=forces[2];
    RPUs.Data.force[3]=forces[3];
   	regWrite(RPUs_BASEADDRESS,RPUs.buffer,sizeof(RPUs.buffer)); 

}

void readKeys(uint8_t *keys)
{
	regRead(KEYS_BASEADDRESS,KEYs.buffer,sizeof(KEYs.buffer));
	memcpy((uint8_t *)keys,KEYs.Data.Keys,sizeof(KEYs.Data.Keys));
}

void SlidersRead(uint8_t *sliders)
{
		regRead(SLIDERS_BASEADDRESS,Sliders.buffer,sizeof(Sliders.buffer));
	  memcpy((uint8_t *)sliders,Sliders.Data.sliders,sizeof(Sliders.Data.sliders));
}

