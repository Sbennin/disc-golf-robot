
#include "main.h"
#include "solenoid.h"

#define D1_HIGH HAL_GPIO_WritePin(digit1_GPIO_Port, digit1_Pin, GPIO_PIN_SET);
#define D1_LOW HAL_GPIO_WritePin(digit1_GPIO_Port, digit1_Pin, GPIO_PIN_RESET);

#define D2_HIGH HAL_GPIO_WritePin(digit2_GPIO_Port, digit2_Pin, GPIO_PIN_SET);
#define D2_LOW HAL_GPIO_WritePin(digit2_GPIO_Port, digit2_Pin, GPIO_PIN_RESET);

#define D3_HIGH HAL_GPIO_WritePin(digit3_GPIO_Port, digit3_Pin, GPIO_PIN_SET);
#define D3_LOW HAL_GPIO_WritePin(digit3_GPIO_Port, digit3_Pin, GPIO_PIN_RESET);

#define D4_HIGH HAL_GPIO_WritePin(digit4_GPIO_Port, digit4_Pin, GPIO_PIN_SET);
#define D4_LOW HAL_GPIO_WritePin(digit4_GPIO_Port, digit4_Pin, GPIO_PIN_RESET);

void SevenSegment_UpdateDigit(uint8_t);

uint8_t segmentNumber[10] = {
		0x3f,
		0x06,
		0x5b,
		0x4f,
		0x66,
		0x6d,
		0x7d,
		0x07,
		0x7f,
		0x67
};

// Call this function as often as possible, each digit only flashes for a few ms
void SevenSegment_UpdateAllDigits(uint32_t value){
	uint8_t tmp_digit1 = value/1000;
	uint8_t tmp_digit2 = ((value/100)%10);
	uint8_t tmp_digit3 = ((value/10)%10);
	uint8_t tmp_digit4 = (value%10);

	SevenSegment_UpdateDigit(segmentNumber[tmp_digit1]);
	D1_LOW;
	HAL_Delay(2);
	D1_HIGH;

	SevenSegment_UpdateDigit(segmentNumber[tmp_digit2]);
	D2_LOW;
	HAL_Delay(2);
	D2_HIGH;

	SevenSegment_UpdateDigit(segmentNumber[tmp_digit3]);
	D3_LOW;
	HAL_Delay(2);
	D3_HIGH;

	SevenSegment_UpdateDigit(segmentNumber[tmp_digit4]);
	D4_LOW;
	HAL_Delay(2);
	D4_HIGH;
}

void SevenSegment_UpdateDigit(uint8_t number){
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, ((number>>0)&0x01));
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, ((number>>1)&0x01));
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, ((number>>2)&0x01));
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, ((number>>3)&0x01));
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, ((number>>4)&0x01));
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, ((number>>5)&0x01));
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, ((number>>6)&0x01));
}

void SevenSegment_Off(){
	// reset segment pins to low/OFF (set to high/ON)
	HAL_GPIO_WritePin(segmentA_GPIO_Port, segmentA_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentB_GPIO_Port, segmentB_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentC_GPIO_Port, segmentC_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentD_GPIO_Port, segmentD_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentE_GPIO_Port, segmentE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentF_GPIO_Port, segmentF_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(segmentG_GPIO_Port, segmentG_Pin, GPIO_PIN_RESET);

	// set all digit (grounds) to high/OFF (reset to low/ON)
	D1_HIGH;
	D2_HIGH;
	D3_HIGH;
	D4_HIGH;
}
