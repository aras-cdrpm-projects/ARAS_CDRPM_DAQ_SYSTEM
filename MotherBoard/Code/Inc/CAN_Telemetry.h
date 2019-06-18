#ifndef __CAN_TELEMETRY
#define  __CAN_TELEMETRY
#include "main.h"
#include "string.h"

extern int16_t accel[3];
extern int16_t gyro[3];
extern int16_t magnetometer[3];
extern int32_t barometer;
void can_stack_init(CAN_HandleTypeDef *hcan);
void can_stack_interrupt_rutine(CAN_HandleTypeDef *hcan);

#endif
