/*
 * state_commands.c
 *
 */

#include "main.h"
#include "communication_utilities.h"
#include "hall_sensor.h"
#include "solenoid.h"
#include "utilities.h"
#include <stdlib.h>

#define BIG_MOTOR_GEAR_RATIO 4
#define COMMAND_DELAY 100 //ms
#define MOTOR_SPEED_TOL 10 //rpm
#define MOTOR_STOP_TOL 5 //rpm
#define LAUNCH_DELAY_DIV 4 //determines when solenoid is triggered in rev cycle, smaller number is sooner

uint16_t Calc_Big_Motor_Speed(uint16_t);
uint16_t Calc_Launch_Delay(uint16_t);
void Motor_Init();

void Arm_Spin_State(uint16_t arm_speed)
{
	Motor_Init();

	uint16_t motor_speed = Calc_Big_Motor_Speed(arm_speed);

	Set_Digital_Speed(motor_speed);
	HAL_Delay(COMMAND_DELAY);
	Start_Motor();
	HAL_Delay(COMMAND_DELAY);
}

uint8_t Arm_Done_Spinning(uint16_t set_arm_speed)
{
	uint16_t set_motor_speed = Calc_Big_Motor_Speed(set_arm_speed);
	uint16_t cur_motor_speed = Verify_Current_Speed();

	if (abs(set_motor_speed - cur_motor_speed) < MOTOR_SPEED_TOL)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t Arm_Stopped()
{
	uint16_t cur_motor_speed = Verify_Current_Speed();

	if (cur_motor_speed < MOTOR_STOP_TOL)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t Arm_Launched_In_Position(uint16_t set_arm_speed)
{
	uint16_t set_motor_speed = Calc_Big_Motor_Speed(set_arm_speed);
	uint16_t launch_delay = Calc_Launch_Delay(set_motor_speed);

	if (Hall_Sensor_Triggered() == 1)
	{
		HAL_Delay(launch_delay);
		Solenoid_Up();
		HAL_Delay(1.0/set_arm_speed*60.0*1000.0); //wait for full revolution to confirm launched
		Solenoid_Down();
		return 1;
	}
	else
	{
		return 0;
	}
}

void Launch_Disc_State(uint16_t set_arm_speed)
{
	uint16_t set_motor_speed = Calc_Big_Motor_Speed(set_arm_speed);
	uint16_t launch_delay = Calc_Launch_Delay(set_motor_speed);

	HAL_Delay(launch_delay);
	//Blue_On();
	//HAL_Delay(500);
	//Blue_Off();
	//Green_On();
	Solenoid_Up();

	//Blue_Off();
	//Green_On();

	//HAL_Delay(500);

	//Green_Off();
	//Red_On();
	//HAL_Delay(launch_delay*LAUNCH_DELAY_DIV); //wait for full rev to confirm launched
	HAL_Delay(1.0/set_arm_speed*60.0*1000.0);
	//Red_Off();
	Solenoid_Down();
}

uint16_t Calc_Big_Motor_Speed(uint16_t arm_speed)
{
	return arm_speed * BIG_MOTOR_GEAR_RATIO;
}

uint16_t Calc_Launch_Delay(uint16_t motor_speed)
{
	uint16_t rev_period = 1000.0/(motor_speed/60.0);
	return rev_period /LAUNCH_DELAY_DIV;
}

void Motor_Init()
{
	Digital_Mode();
	HAL_Delay(COMMAND_DELAY);
	Set_Counterclockwise_Direction();
	HAL_Delay(COMMAND_DELAY);
	Coast();
	HAL_Delay(COMMAND_DELAY);
}
