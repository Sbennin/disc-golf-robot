/*
 * LCD_utilities.h
 */

#ifndef INC_LCD_UTILITIES_H_
#define INC_LCD_UTILITIES_H_

#include "main.h"

void Init_LCD();
void Disp_Pic(unsigned char *lcd_string);
void Clear_LCD();
void Black_LCD();
void Disp_Text_Small(unsigned char* text, uint8_t length);

unsigned char DiscBotLogo [] = { //16x32= 512
0x00, 0xFE, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22,
0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02,
0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x70, 0x18, 0x1C, 0x0C,
0x0E, 0x06, 0x06, 0x07, 0x03, 0x03, 0x03, 0x03, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x07,
0x06, 0x06, 0x0E, 0x0C, 0x1C, 0x18, 0xF0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x04, 0x04, 0x04, 0x08, 0xF0, 0x00, 0x04, 0x04, 0xFC, 0x04,
0x04, 0x00, 0x00, 0x18, 0x24, 0x24, 0x44, 0x80, 0x00, 0x00, 0xF0, 0x08, 0x04, 0x04, 0x04, 0x00,
0x00, 0xFC, 0x24, 0x24, 0x24, 0xD8, 0x00, 0xF0, 0x08, 0x04, 0x04, 0x04, 0x0C, 0xF0, 0x00, 0x04,
0x04, 0xFC, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x30, 0x30, 0x30, 0x60,
0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60,
0x60, 0x60, 0x60, 0x60, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x18, 0x18, 0x18, 0x1C,
0x0C, 0x0C, 0x0E, 0x06, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
0x40, 0x40, 0x40, 0x40, 0x40, 0x43, 0x42, 0x42, 0x42, 0x41, 0x40, 0x40, 0x42, 0x42, 0x43, 0x42,
0x42, 0x40, 0x40, 0x42, 0x42, 0x42, 0x42, 0x41, 0x40, 0x40, 0x41, 0x43, 0x42, 0x42, 0x42, 0x40,
0x40, 0x43, 0x42, 0x42, 0x42, 0x41, 0x40, 0x40, 0x43, 0x42, 0x42, 0x42, 0x41, 0x40, 0x40, 0x40,
0x40, 0x43, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40,
0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#endif /* INC_LCD_UTILITIES_H_ */
