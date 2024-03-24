/*
 * hall_sensor.c
 *
 */

#include "main.h"

uint8_t Hall_Sensor_Triggered()
{
	if ( HAL_GPIO_ReadPin(HALL_SENSOR_GPIO_Port, HALL_SENSOR_Pin) == GPIO_PIN_RESET)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint32_t Read_Motor_Speed()
{
	//TODO implement
}
