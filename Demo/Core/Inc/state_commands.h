/*
 * state_commands.h
 *
 */

#ifndef INC_STATE_COMMANDS_H_
#define INC_STATE_COMMANDS_H_

void Arm_Spin_State(uint16_t);
uint8_t Arm_Done_Spinning(uint16_t);
uint8_t Arm_Launched_In_Position();
uint8_t Arm_Stopped();
void Launch_Disc_State(uint16_t);
void Start_Scanning();


#endif /* INC_STATE_COMMANDS_H_ */
