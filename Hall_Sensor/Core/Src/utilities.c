/*
 * utilities.c
 *
 */

#include "utilities.h"
#include <stdio.h>

uint16_t Array_To_Int(char array[])
{
	uint16_t i;
	sscanf(array, "%hd", &i);
	return i;
}

void Red_On()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}

void Red_Off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
}

void Green_On()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
}

void Green_Off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
}

void Blue_On()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
}

void Blue_Off()
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void LED_Num(uint8_t i)
{
	if (i == 4 || i == 5 ||i == 6 || i == 7)
	{
		Blue_On();
	}
	else
	{
		Blue_Off();
	}

	if (i == 2 || i == 3 ||i == 6 || i == 7)
	{
		Green_On();
	}
	else
	{
		Green_Off();
	}

	if (i == 1 || i == 3 ||i == 5 || i == 7)
	{
		Red_On();
	}
	else
	{
		Red_Off();
	}
}
