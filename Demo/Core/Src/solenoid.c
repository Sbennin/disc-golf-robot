/*
 * solenoid.c
 *
 */

#include "main.h"
#include "solenoid.h"

void Solenoid_Up()
{
	HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_SET);
}

void Solenoid_Down()
{
	HAL_GPIO_WritePin(Solenoid_GPIO_Port, Solenoid_Pin, GPIO_PIN_RESET);
}
