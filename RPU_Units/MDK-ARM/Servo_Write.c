#include "Servo_Write.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"



extern DAC_HandleTypeDef hdac;

void servoWrite(float value)
{
	//value=-value;
	if(value>limit)
		value=limit;
	else if(value<-limit)
		value=-limit;
//	else if(value<0 && value>-100)
	//	value=-100;
//	else if(value>0 && value<100)
	//	value=100;
	if(value>0){
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t )value);
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t )0);
	}else if(value<0){
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t )0);
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t )-value);
	}else{
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,(uint32_t )0);
		HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,(uint32_t )0);
	}
}
