#include "stm32f7xx_hal.h"
#include "string.h"
#define IMP_BASEADDRESS     0
#define IMU_BASEADDRESS     30
#define RPUs_BASEADDRESS    68
#define KYES_BASEADDRESS    102
#define SLIDERS_BASEADDRESS 110

extern volatile uint8_t spi_regs[127];
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
    memcpy((uint8_t *)data,(uint8_t *)&spi_regs[Address],len);
}


void regWrite(uint8_t Address,uint8_t *data,int len)
{
    memcpy((uint8_t *)&spi_regs[Address],(uint8_t *)data,len);

}
void spi_stack_init(SPI_HandleTypeDef *spi){
	spi->Instance->CR2|=(1<<6); //Enable the interrupt 
	memset((uint8_t*)spi_regs,0x00,sizeof(spi_regs));
}

void spi_stack_stateMacine(SPI_HandleTypeDef *spi)
{
	uint8_t data= *(uint8_t *)&(spi->Instance->DR);
		if(spiByteCounter==0) //Is it the first Byte? If yes, Exteract the Address and RW cmd
		{
				address=data&0x7f;
				R_W = (data>>7)&0x1;
			if(R_W==1)	//If Read command is issued, the first byte of the regs should be loaded into the DR Reg to be transmitted
				  *(uint8_t *)&(spi->Instance->DR)=spi_regs[address];
		}else if(R_W==1)
		{
					*(uint8_t *)&(spi->Instance->DR)=spi_regs[address]; //If write cmd is asserted, Save the bytes following the first command and RW byte.
		}else
		{
				  spi_regs[address-1]=data;
		}
		address++;
		spiByteCounter++;
}

void spi_stack_CS_Mng(SPI_HandleTypeDef *spi)
{
	uint8_t data;
		while(spi->Instance->SR&0x1)
		 data= *(uint8_t *)&(spi->Instance->DR);
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) //If the interrupt is generated due to a rising edge, The SPI module and the software SS signals
																					 //Have to be set
		{
			spi->Instance->CR1|=(1<<8); // Set the Software controled SS Signal
			spiByteCounter=0;						 // Reset the Byte counter
			spi->Instance->CR1&=~(1<<6);// Disable the SPI Module
		}else{
		  spiByteCounter=0;
			spi->Instance->CR1&=~(1<<8);//Assert the NSS signal(This causes the MOSI Pin to be set to output mode instead of being a Tri-state signal)
			spi->Instance->CR1|=(1<<6); //Enable the SPI Module

		}
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
    a[0]=IMU.Data.a[0];
    a[1]=IMU.Data.a[1];
    a[2]=IMU.Data.a[2];
    g[0]=IMU.Data.g[0];
    g[1]=IMU.Data.g[1];
    g[2]=IMU.Data.g[2];
    m[0]=IMU.Data.m[0];
    m[1]=IMU.Data.m[1];
    m[2]=IMU.Data.m[2];
   	regWrite(IMU_BASEADDRESS,IMU.buffer,sizeof(IMU.buffer)); 

}

void RPUs_Write(int32_t *encoders, int32_t *forces)
{
    encoders[0]=RPUs.Data.Enc[0];
    encoders[1]=RPUs.Data.Enc[1];
    encoders[2]=RPUs.Data.Enc[2];
    encoders[3]=RPUs.Data.Enc[3];
    forces[0]=RPUs.Data.force[0];
    forces[1]=RPUs.Data.force[1];
    forces[2]=RPUs.Data.force[2];
    forces[3]=RPUs.Data.force[3];
   	regWrite(RPUs_BASEADDRESS,RPUs.buffer,sizeof(RPUs.buffer)); 

}

void readKeys(uint8_t *keys)
{
	memcpy((uint8_t *)KEYs.buffer,(uint8_t *)&spi_regs[KYES_BASEADDRESS],sizeof(KEYs.buffer));
	memcpy((uint8_t *)keys,KEYs.Data.Keys,sizeof(KEYs.Data.Keys));
}

void SlidersRead(uint8_t *sliders)
{

		memcpy((uint8_t *)Sliders.buffer,(uint8_t *)&spi_regs[SLIDERS_BASEADDRESS],sizeof(Sliders.buffer));
	  memcpy((uint8_t *)sliders,Sliders.Data.sliders,sizeof(Sliders.Data.sliders));
}

