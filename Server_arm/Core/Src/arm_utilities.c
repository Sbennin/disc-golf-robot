/*
 * arm_utilities.c
 *
 */


#include "arm_utilities.h"
#include "main.h"
#include "app_ble.h"
#include "dbg_trace.h"

/* Private define ------------------------------------------------------------*/
#define PPR 7
#define GEAR_RATIO 3.7 * 3.0
#define PI_USED 0

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Rot_CW_PWM(uint32_t);
void Rot_CCW_PWM(uint32_t);
uint16_t Read_Speed();
void Update_PWM(uint32_t);
void Motor_Stopped_Complete();
void Motor_Pending_Complete();
void Motor_Done_Complete();

/* Private user code ---------------------------------------------------------*/
void Rot_CW_PWM(uint32_t duty) //facing motor end
{
	Set_CW();
	Update_PWM(duty);
}

void Rot_CCW_PWM(uint32_t duty)
{
	Set_CCW();
	Update_PWM(duty);

}

void Set_CW()
{
	HAL_GPIO_WritePin(SYS_ENABLE_GPIO_Port, SYS_ENABLE_Pin, GPIO_PIN_RESET);

	Update_PWM(0);
	HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(SYS_ENABLE_GPIO_Port, SYS_ENABLE_Pin, GPIO_PIN_SET);
	HAL_Delay(500); //so motor doesn't jerk
}

void Set_CCW()
{
	HAL_GPIO_WritePin(SYS_ENABLE_GPIO_Port, SYS_ENABLE_Pin, GPIO_PIN_RESET);

	Update_PWM(0);
	HAL_GPIO_WritePin(DIR_A_GPIO_Port, DIR_A_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DIR_B_GPIO_Port, DIR_B_Pin, GPIO_PIN_RESET);


	HAL_GPIO_WritePin(SYS_ENABLE_GPIO_Port, SYS_ENABLE_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
}

void Stop_Motor()
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	Update_PWM(0);
	Motor_Stopped_Complete();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

uint16_t Get_Speed() // does not look at direction
{
	uint32_t total = 0;
	uint16_t avg = 0;
	uint8_t readings = 3;
	for (int i = 0; i < readings; i++)
	{
		total += Read_Speed();
		HAL_Delay(2); //Offset
	}

	avg = total/readings;
	return avg;
}

uint16_t Read_Speed() //rpm
{
	//return 5;
	//TIM1->CR1 = 1;
	//uint8_t Test[] = "Starting speed reading\r\n";
	//HAL_UART_Transmit(&huart1,Test, sizeof(Test),10);

	uint32_t period = 50;
	uint16_t speed = 0;
	uint32_t start_counter = Get_Counter();

	//char Test3[50];
	//int size3 = sprintf(Test3, "start counter= %lu\r\n", start_counter);
	//HAL_UART_Transmit(&huart1,(uint8_t*)Test3, size3,10);

	HAL_Delay(period);
	uint32_t end_counter = Get_Counter();

	//char Test2[50];
	//int size2 = sprintf(Test2, "end counter=   %lu\r\n", end_counter);
	//HAL_UART_Transmit(&huart1,(uint8_t*)Test2, size2,10);

	//APP_DBG_MSG("\r\n\r** start: %lu, end: %lu\n", start_counter, end_counter);

	if (start_counter == end_counter)
	{
		speed = 0;
	}
	else if (start_counter > end_counter)
	{
		speed = (start_counter - end_counter)*(1000.0/period)*60.0/4/PPR/GEAR_RATIO;
	}
	else //overflow occurred
	{
		speed = (4294967295 - end_counter + start_counter)*(1000.0/period)*60.0/4/PPR/GEAR_RATIO;
	}
	return speed;
}

void Set_Speed(uint16_t goal) //TODO detect failure
{
#if (PI_USED == 1)
	Set_CW();
	float Kp = 0.03;
	float Ki = 0.04;
	int period = 100;
	int stable = 0;
	int stable_threshold = 5;
	int failures = 0; //to check if it never converges
	int failure_threshold = 50;
	int I = 0;

	APP_DBG_MSG("\r\n\r** in Set_Speed at start \n");

	do
	{
		uint16_t current = Get_Speed();

		//APP_DBG_MSG("\r\n\r** Current Speed: %d \n", current);

		  //char Test[50];
		  //int size = sprintf(Test, "cur speed: %d\r\nfailures: %d\r\n", current, failures);
		  //UART_Transmit(Test);

		int error = goal - current;

		int P = error*Kp;
		I = I + error*Ki;
		int duty = P + I;

		APP_DBG_MSG("\r\n\r** Current Speed: %d, unsaturated duty: %d \n", current, duty);
		if (duty < 0)
		{
			duty = 0;
		}
		else if (duty > 100)
		{
			duty = 100;
		}

		Update_PWM(duty); //TODO change to higher resolution
		HAL_Delay(period);
		if (error < 10 && error > -10)
		{
			stable++;
		}
		else
		{
			stable = 0;
		}
		failures++;

	} while(stable < stable_threshold && failures < failure_threshold);
	Motor_Done_Complete();
#else
	Set_CW();
	int duty = 100*(goal/1200.0);
	if (duty > 100)
	{
		duty = 100;
	}
	Update_PWM(duty);
	Motor_Done_Complete();
#endif
}

void Update_PWM(uint32_t duty)
{
	//APP_DBG_MSG("\r\n\r** in Update_PWM, duty: %lu\n", duty);
	uint32_t CCR = (TIM16->ARR)*(duty/100.0); //TODO checnge ARR to higher resolution
	//APP_DBG_MSG("\r\n\r** in Update_PWM, CCR: %lu\n", CCR);
	//APP_DBG_MSG("\r\n\r** CCR: %lu \n", CCR);
	Set_CCR(CCR);
}

void Motor_Stopped_Complete()
{
	APP_BLE_Key_Motor_Stopped_Action();
}

void Motor_Pending_Complete()
{
	APP_BLE_Key_Motor_Pending_Action();
}

void Motor_Done_Complete()
{
	APP_BLE_Key_Motor_Done_Action();
}

void Red_On()
{
	HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_SET);
}

void Red_Off()
{
	HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);
}

void Green_On()
{
	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_SET);
}

void Green_Off()
{
	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);
}

void Blue_On()
{
	HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_SET);
}

void Blue_Off()
{
	HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);
}

