#ifndef __SPI_STACK
#define __SPI_STACK
#include "string.h"
#include "stm32f1xx_hal.h"
#define STATUS_ADDRESS		0
#define FORCE_ADDRESS			4
#define ENCODER_ADDRESS		8
#define DAC_ADDRESS				12
#define SPI_LINK_STATUS		16
void spi_stack_init(SPI_HandleTypeDef *spi);
void spi_stack_CS_Mng(SPI_HandleTypeDef *spi);
void spi_dma_rx_complete_rutine(void);
void spi_out_shadow_update(void);
void spi_set_register(int address,int32_t val);
int32_t spi_get_register(int address);
void spi_set_rx_chache_register(int address,int32_t val);

#endif
