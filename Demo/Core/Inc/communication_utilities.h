/*
 * communication_utilities.h
 *
 */

#ifndef INC_COMMUNICATION_UTILITIES_H_
#define INC_COMMUNICATION_UTILITIES_H_

/* Private includes ----------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/


/* Exported macro ------------------------------------------------------------*/
#define BIG_MOTOR_GEAR_RATIO (9.75/2.4)
#define COMMAND_DELAY 100 //ms
#define MOTOR_SPEED_TOL 10 //rpm
#define MOTOR_STOP_TOL 5 //rpm
#define LAUNCH_DELAY_DIV 4 //determines when solenoid is triggered in rev cycle, smaller number is sooner

/* Exported functions prototypes ---------------------------------------------*/
uint16_t Verify_Test();
void Analog_Mode();
void Digital_Mode();
void Set_To_Factory_Default();
void Set_Inital_Value(uint8_t);
void Set_Gain_Value(uint8_t);
void Set_Integrator_Constant(uint8_t);
void Set_Proportional_Constant(uint8_t);
void Set_Digital_Speed(uint16_t);
void Start_Motor();
char Verify_Direction();
uint16_t Verify_Digital_Speed();
uint8_t Verify_Inital_Value();
uint8_t Verify_Gain_Value();
uint8_t Verify_Integrator_Constant();
uint8_t Verify_Proportional_Constant();
uint16_t Verify_Current_Speed();
char Verify_Mode();
uint16_t Verify_Analog_Speed_Min();
uint16_t Verify_Analog_Speed_Max();
uint8_t Verify_Numer_Of_Poles();
uint16_t Verify_Set_Speed();
char Verify_Brake_Setting();
void Verify_All_Parameters();
void Coast();
void Hard_Brake();
void Set_Clockwise_Direction();
void Set_Counterclockwise_Direction();
uint8_t Verify_Address();
void Set_Address(uint8_t);
void Set_Analog_Speed_Min(uint16_t);
void Set_Analog_Speed_Max(uint16_t);

/* Private defines -----------------------------------------------------------*/


#endif /* INC_COMMUNICATION_UTILITIES_H_ */
