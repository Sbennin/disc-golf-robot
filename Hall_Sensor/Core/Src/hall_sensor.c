/*
 * hall_sensor.c
 *
 */

#include "main.h"

uint8_t Hall_Sensor_Triggered()
{
	if ( HAL_GPIO_ReadPin(Hall_Sensor_GPIO_Port, Hall_Sensor_Pin) == GPIO_PIN_RESET)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
