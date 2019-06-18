#ifndef __SPI_STACK
#define __SPI_STACK
#include "string.h"
#include "stm32f7xx_hal.h"
extern volatile uint8_t spi_regs[127];

void IMP_Write(float *pos,float *q);
void IMU_Write(int32_t *a,int32_t *g,int32_t *m);
void RPUs_Write(int32_t *encoders,int32_t *forces);
void readKeys(uint8_t *keys);
void SlidersRead(uint8_t *sliders);
void spi_stack_init(void); // To be called after the spi init functions
void spi_stack_stateMacine(SPI_HandleTypeDef *spi);// To be called in the spi interrupt rutien
void spi_stack_CS_Mng(void);// To be called in the GPIO interrupt rutien
void spi_dma_rx_complete_rutine(void);

#endif
