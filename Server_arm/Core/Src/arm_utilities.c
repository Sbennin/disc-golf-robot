/*
 * arm_utilities.c
 *
 */


#include "arm_utilities.h"

/* Private define ------------------------------------------------------------*/
#define PPR 7
#define GEAR_RATIO 3.7

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
void Rot_CW_PWM(uint32_t);
void Rot_CCW_PWM(uint32_t);
uint16_t Read_Speed();
void Update_PWM(uint32_t);

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
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	Update_PWM(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(500); //so motor doesn't jerk
}

void Set_CCW()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	Update_PWM(0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(500);
}

void Stop_Motor()
{
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	Update_PWM(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

uint16_t Get_Speed() // does not look at direction
{
	uint32_t total = 0;
	uint16_t avg = 0;
	for (int i = 0; i < 8; i++)
	{
		total += Read_Speed();
		HAL_Delay(2); //Offset
	}

	avg = total/8;
	return avg;
}

uint16_t Read_Speed() //rpm
{
	//return 5;
	//TIM1->CR1 = 1;
	//uint8_t Test[] = "Starting speed reading\r\n";
	//HAL_UART_Transmit(&huart1,Test, sizeof(Test),10);

	uint32_t period = 100;
	uint16_t speed = 0;
	uint32_t start_counter = __HAL_TIM_GET_COUNTER(&htim2);

	//char Test3[50];
	//int size3 = sprintf(Test3, "start counter= %lu\r\n", start_counter);
	//HAL_UART_Transmit(&huart1,(uint8_t*)Test3, size3,10);

	HAL_Delay(period);
	uint32_t end_counter = __HAL_TIM_GET_COUNTER(&htim2);

	//char Test2[50];
	//int size2 = sprintf(Test2, "end counter=   %lu\r\n", end_counter);
	//HAL_UART_Transmit(&huart1,(uint8_t*)Test2, size2,10);

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

void Set_Speed(uint32_t goal)
{
	float Kp = 0.03;
	float Ki = 0.04;
	int period = 100;
	int stable = 0;
	int stable_threshold = 5;
	int failures = 0; //to check if it never converges
	int failure_threshold = 50;
	int I = 0;

	do
	{
		int current = Get_Speed();

		  char Test[50];
		  int size = sprintf(Test, "cur speed: %d\r\nfailures: %d\r\n", current, failures);
		  HAL_UART_Transmit(&huart1,(uint8_t*)Test, size,10);

		int error = goal - current;

		int P = error*Kp;
		I = I + error*Ki;
		int duty = P + I;
		if (duty < 0)
		{
			duty = 0;
		}
		else if (duty > 100)
		{
			duty = 100;
		}

		Update_PWM(duty);
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
}

void Update_PWM(uint32_t duty)
{
	uint32_t CCR = (TIM1->ARR)*(duty/100.0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, CCR);
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
