#ifndef __TIME_KEEPER
#define __TIME_KEEPER
#include "stm32f7xx_hal.h"

void timeManagerInit(TIM_HandleTypeDef *tmr);
void time_keeper_interrupt_rutine(void);
uint32_t Micros(void);
void RPU_Tick(void);
void IMU_Tick(void);
void Camera_Tick(void);
uint32_t RPU_TimeStamp(void);
uint32_t IMU_TimeStamp(void);
uint32_t Camera_TimeStamp(void);

#endif
