/*
 * arm_utilities.h
 *
 */

#ifndef INC_ARM_UTILITIES_H_
#define INC_ARM_UTILITIES_H_

#include <stdint.h>


/* Exported functions ---------------------------------------------*/
uint16_t Get_Speed();
void Set_Speed(uint16_t);
void Stop_Motor();
void Set_CW();
void Set_CCW();
void Red_On();
void Red_Off();
void Green_On();
void Green_Off();
void Blue_On();
void Blue_Off();

#endif /* INC_ARM_UTILITIES_H_ */
