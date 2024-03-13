/*
 * LCD_utilities.c
 *
 */

//#include "LCD_utilities.h"
#include "main.h"

void comm_write(unsigned char command);
void data_write(unsigned char data);
void comm_ADC_Select();
void comm_Display_Off();
void comm_Display_On();
void comm_Com_Output_Scan_Reverse();
void comm_LCD_Bias_1_9();
void comm_Power_Control();
void comm_Resistor_Ratio();
void comm_Volume_Mode();
void comm_Volume_Register();
void comm_Display_Start_Addr_0();
void comm_Col_Upper_Addr();
void comm_Col_Lower_Addr();
void comm_Page_Addr(unsigned char addr);


void Init_LCD()
{
	void comm_ADC_Select();
	HAL_Delay(5);
	void comm_Display_Off();
	HAL_Delay(5);
	void comm_Com_Output_Scan_Reverse();
	HAL_Delay(5);
	void comm_LCD_Bias_1_9();
	HAL_Delay(5);
	void comm_Power_Control();
	HAL_Delay(5);
	void comm_Resistor_Ratio();
	HAL_Delay(5);
	void comm_Volume_Mode();
	HAL_Delay(5);
	void comm_Volume_Register();
	HAL_Delay(5);
	void comm_Display_On();
}

void Disp_Pic(unsigned char *lcd_string)
{
	//LCD organized into 4 pages(rows), each of 128 pixels wide and 8 pixels high.
	//Each element in lcd_string is a vertical column of 8 pixels.
	//pages are populated left to right in columns of 8 pixels
    unsigned int i,j;
    unsigned char page = 0x00; //starting page address at 0xB0 + 0

    comm_Display_Off();
    HAL_Delay(5);
    comm_Display_Start_Addr_0();
    HAL_Delay(5);

    for(i=0; i<4; i++) //32 pixel high display / 8 pixels per page = 4 pages
    {
    	comm_Page_Addr(page);       //set current page address
    	HAL_Delay(5);
    	comm_Col_Upper_Addr();
    	HAL_Delay(5);
    	comm_Col_Lower_Addr();
    	HAL_Delay(5);

        for(j=0; j<128; j++) //128 columns wide
        {
        	data_write(*lcd_string);    //send picture data
        	HAL_Delay(5);
        	lcd_string++; //goes to next element/column
        }

        page++;         //after 128 columns, go to next page
    }
    comm_Display_On();
    HAL_Delay(5);
}

void Clear_LCD()
{
    unsigned int i,j;
    unsigned char page = 0x00;
    comm_Display_Off();
    HAL_Delay(5);
    comm_Display_Start_Addr_0();
    HAL_Delay(5);

    for(i = 0; i < 4; i++) //32pixel display / 8 pixels per page = 4 pages
    {
    	comm_Page_Addr(page);       //set current page address
    	HAL_Delay(5);
    	comm_Col_Upper_Addr();
    	HAL_Delay(5);
    	comm_Col_Lower_Addr();
    	HAL_Delay(5);

        for(j = 0; j < 128; j++) //128 columns wide
        {
    	    data_write(0x00);    //send picture data
    	    HAL_Delay(5);
        }
        page++;         //after 128 columns, go to next page
    }
    comm_Display_On();
    HAL_Delay(5);
}

void Black_LCD()
{
    unsigned int i,j;
    unsigned char page = 0x00;
    comm_Display_Off();
    HAL_Delay(5);
    comm_Display_Start_Addr_0();
    HAL_Delay(5);

    for(i = 0; i < 4; i++) //32pixel display / 8 pixels per page = 4 pages
    {
    	comm_Page_Addr(page);       //set current page address
    	HAL_Delay(5);
    	comm_Col_Upper_Addr();
    	HAL_Delay(5);
    	comm_Col_Lower_Addr();
    	HAL_Delay(5);

        for(j = 0; j < 128; j++) //128 columns wide
        {
    	    data_write(0xFF);    //send picture data
    	    HAL_Delay(5);
        }
        page++;         //after 128 columns, go to next page
    }
    comm_Display_On();
    HAL_Delay(5);
}

void data_write(unsigned char data) //Data Output Serial Interface
{
	unsigned int n;

	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_SET); //data type

	for(n=0; n<8; n++) //each pixel in the column
	{
		if((data&0x80)==0x80)
		{
			HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, GPIO_PIN_RESET);
		}


		while(0);
		HAL_Delay(5);
		data = (data<<1);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
		while(0);
		HAL_Delay(5);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
		while(0);
		HAL_Delay(5);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

void comm_write(unsigned char command)
{
	unsigned int n;

	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET); //CS //activating LCD control
	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, GPIO_PIN_RESET); //RS //instruction/setting type

	for(n=0; n<8; n++)
	{
		//sends each bit of the command 1 at a time, starting at most significant (left to right)
		if((command&0x80)==0x80)
		{
			HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, GPIO_PIN_SET); //SI
		}
		else
		{
			HAL_GPIO_WritePin(SI_GPIO_Port, SI_Pin, GPIO_PIN_RESET);
		}


		while(0);
		HAL_Delay(5);
		command = (command<<1);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET); //SC
		while(0);
		HAL_Delay(5);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_SET);
		while(0);
		HAL_Delay(5);
		HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

void comm_ADC_Select()
{
	comm_write(0xA0); //see manual to get binary commands
}

void comm_Display_Off()
{
	comm_write(0xAE);
}

void comm_Display_On()
{
	comm_write(0xAF);
}

void comm_Com_Output_Scan_Reverse()
{
	comm_write(0xC8);
}

void comm_LCD_Bias_1_9()
{
	comm_write(0xA2);
}

void comm_Power_Control()
{
	comm_write(0x2F);
}

void comm_Resistor_Ratio()
{
	comm_write(0x21);
}

void comm_Volume_Mode()
{
	comm_write(0x81);
}

void comm_Volume_Register()
{
	comm_write(0x20);
}

void comm_Display_Start_Addr_0()
{
	comm_write(0x40);
}

void comm_Col_Upper_Addr()
{
	comm_write(0x10);
}

void comm_Col_Lower_Addr()
{
	comm_write(0x00);
}

void comm_Page_Addr(unsigned char addr)
{
	comm_write(0xB0 + addr);
}
