#include "stm32f7xx_hal.h"

uint64_t tick=0,rpu_tick=0,camera_tick=0,imu_tick=0;
uint32_t tick_index=0;
TIM_HandleTypeDef *timer;
void  timeManagerInit(TIM_HandleTypeDef *tmr)
{
	timer=tmr;
	HAL_TIM_Base_Start_IT(timer);
}
void time_keeper_interrupt_rutine(void)
{
	tick_index++;
}
uint32_t Micros(void)
{
	return (tick_index*65536)+(timer->Instance->CNT);
}

void RPU_Tick(void)
{
	rpu_tick=Micros();
}

void IMU_Tick(void)
{
	imu_tick=Micros();
}

void Camera_Tick(void)
{
	camera_tick=Micros();
}
uint32_t RPU_TimeStamp(void)
{
	return rpu_tick;
}
uint32_t IMU_TimeStamp(void)
{
	return imu_tick;
}
uint32_t Camera_TimeStamp(void)
{
	return camera_tick;
}
