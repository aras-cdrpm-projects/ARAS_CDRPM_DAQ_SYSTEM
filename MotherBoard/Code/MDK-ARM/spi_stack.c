#include "stm32f7xx_hal.h"
#include "string.h"
#define IMP_BASEADDRESS     0
#define IMU_BASEADDRESS     30
#define RPUs_BASEADDRESS    68
#define KEYS_BASEADDRESS    102
#define SLIDERS_BASEADDRESS 110
#define BUFF_SIZE 128
extern SPI_HandleTypeDef hspi1;
uint8_t spi_rxBuff[BUFF_SIZE],spi_txBuff[BUFF_SIZE];

uint8_t spi_shadow_in[BUFF_SIZE] ,spi_shadow_out[BUFF_SIZE];
uint8_t spi_test_out[BUFF_SIZE];



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

void spi_out_shadow_update(void)
{
			memcpy((uint8_t*)spi_shadow_out,spi_txBuff,BUFF_SIZE);//copy the shadow buffer to the rx buffer
			//spi_shadow_out[BUFF_SIZE-1]=0x55;// the vertification header 
}
void gui_spi_set_register(int address,int32_t val)
{
	union{
	int32_t val;
	uint8_t buff[4];
	}packet;
	packet.val=val;
	memcpy((uint8_t*)&spi_txBuff[address],(uint8_t*)packet.buff,4);
}

int32_t gui_spi_get_register(int address)
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



void spi_stack_init(SPI_HandleTypeDef *spi){
	memset((uint8_t*)spi_rxBuff,0x00,sizeof(spi_rxBuff));
	memset((uint8_t*)spi_txBuff,0x00,sizeof(spi_rxBuff));
	memset((uint8_t*)spi_rxBuff,0x00,sizeof(spi_shadow_out));
	memset((uint8_t*)spi_txBuff,0x00,sizeof(spi_shadow_in));
	spi_shadow_out[BUFF_SIZE-1]=0x55;// the vertification header 
	spi->Instance->CR1|=(1<<8); // Put the deevice in slave mode

}
void spi_stack_CS_Mng(SPI_HandleTypeDef *spi){
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)) //If the interrupt is generated due to a rising edge, The SPI module and the software SS signals
																					 //Have to be set
		{
			spi->Instance->CR1|=(1<<8); //SET the SW NSS Signal
			spi->Instance->CR1&=~(1<<6);//Desable The SPI Module so that the Buffers get flused
			//rearm the DMA and SPI System
			HAL_SPI_DMAStop(&hspi1);
			//if(spi_shadow_in[BUFF_SIZE-1]==0x55) // The received data is valid if the last byte is 0x55
				memcpy((uint8_t*)spi_rxBuff,spi_shadow_in,BUFF_SIZE);//copy the shadow buffer to the rx buffer
			HAL_SPI_TransmitReceive_DMA(&hspi1,spi_shadow_out,spi_shadow_in,128); //Start The DMA in circular Mode and assign the buffers
		}else{
			spi->Instance->CR1&=~(1<<8); //Restet the SW NSS Signal
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

