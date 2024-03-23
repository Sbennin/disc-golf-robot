/*
 * LCD_utilities.c
 *
 */

//#include "LCD_utilities.h"
#include "main.h"

const uint8_t PIXEL_WIDTH = 128;
const uint16_t MAX_DISP_LENGTH = 128*4;

//8 pixels high
typedef struct{
	unsigned char ASCII;
	uint8_t width;
	unsigned char data[5];
}letter_small_t;

//TODO define numbers
const letter_small_t LETTER_SPACE_SMALL = {0, 1, {0x00,0x00, 0x00, 0x00,0x00}};
const letter_small_t WORD_SPACE_SMALL = {' ', 3, {0x00, 0x00, 0x00,0x00, 0x00}};

const letter_small_t A_SMALL = {'A', 4, {0x3C, 0x50, 0X50, 0X3C, 0x00}};
const letter_small_t B_SMALL = {'B', 4, {0x7C, 0x54, 0X54, 0X28, 0x00}};
const letter_small_t C_SMALL = {'C', 4, {0x38, 0x44, 0X44, 0X28, 0x00}};
const letter_small_t D_SMALL = {'D', 4, {0x7C, 0x44, 0X44, 0X38, 0x00}};
const letter_small_t E_SMALL = {'E', 4, {0x7c, 0x54, 0x54, 0x44, 0x00}};
const letter_small_t F_SMALL = {'F', 4, {0x7c, 0x50, 0x50, 0x40, 0x00}};
const letter_small_t G_SMALL = {'G', 4, {0x38, 0x44, 0x54, 0x18, 0x00}};
const letter_small_t H_SMALL = {'H', 4, {0x7c, 0x10, 0x10, 0x7c, 0x00}};
const letter_small_t I_SMALL = {'I', 3, {0x44, 0x7c, 0x44, 0x00, 0x00}};
const letter_small_t J_SMALL = {'J', 4, {0x08, 0x04, 0x04, 0x78, 0x00}};
const letter_small_t K_SMALL = {'K', 4, {0x7c, 0x10, 0x28, 0x44, 0x00}};
const letter_small_t L_SMALL = {'L', 4, {0x7c, 0x04, 0x04, 0x04, 0x00}};
const letter_small_t M_SMALL = {'M', 5, {0x7c, 0x20, 0x10, 0x20, 0x7c}};
const letter_small_t N_SMALL = {'N', 4, {0x7c, 0x20, 0x10, 0x7c, 0x00}};
const letter_small_t O_SMALL = {'O', 4, {0x38, 0x44, 0x44, 0x38, 0x00}};
const letter_small_t P_SMALL = {'P', 4, {0x7c, 0x50, 0x50, 0x20, 0x00}};
const letter_small_t Q_SMALL = {'Q', 4, {0x38, 0x44, 0x48, 0x34, 0x00}};
const letter_small_t R_SMALL = {'R', 4, {0x7c, 0x50, 0x58, 0x24, 0x00}};
const letter_small_t S_SMALL = {'S', 4, {0x24, 0x54, 0x54, 0x48, 0x00}};
const letter_small_t T_SMALL = {'T', 5, {0x40, 0x40, 0x7c, 0x40, 0x40}};
const letter_small_t U_SMALL = {'U', 4, {0x78, 0x04, 0x04, 0x78, 0x00}};
const letter_small_t V_SMALL = {'V', 5, {0x60, 0x18, 0x04, 0x18, 0x60}};
const letter_small_t W_SMALL = {'W', 5, {0x7c, 0x08, 0x10, 0x08, 0x7c}};
const letter_small_t X_SMALL = {'X', 4, {0x6c, 0x10, 0x10, 0x6c, 0x00}};
const letter_small_t Y_SMALL = {'Y', 5, {0x60, 0x10, 0x1c, 0x10, 0x60}};
const letter_small_t Z_SMALL = {'Z', 4, {0x4c, 0x54, 0x64, 0x44, 0x00}};

const letter_small_t letter_small_table [] = {
		WORD_SPACE_SMALL, A_SMALL, B_SMALL, C_SMALL, D_SMALL, E_SMALL, F_SMALL, G_SMALL,
		H_SMALL, I_SMALL, J_SMALL, K_SMALL, L_SMALL, M_SMALL, N_SMALL, O_SMALL, P_SMALL,
		Q_SMALL, R_SMALL, S_SMALL, T_SMALL, U_SMALL, V_SMALL, W_SMALL, X_SMALL, Y_SMALL,
		Z_SMALL};

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
uint8_t Find_Char_Index(unsigned char, uint8_t);


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

void Disp_Text_Small(unsigned char* text, uint8_t length)
{
	//TODO split lines by word instead of letter
	//TODO detect message is too long
	//TODO send data directly to LCD rather then calling Disp_Pic, faster
	if (length == 0){return;}

	unsigned char Disp [512];
	uint16_t Disp_index = 0;
	uint8_t remaining_width = PIXEL_WIDTH;

	for (int i = 0; i < length; i++)
	{
		unsigned char new_letter = *text;
		uint8_t table_index = Find_Char_Index(new_letter, 1);
		if (table_index == 255)
		{
			continue; //skip past this letter
		}
		letter_small_t table_letter = letter_small_table[table_index];

		if(remaining_width < table_letter.width + 1) //need to go to next line, fill in extra spaces
		{
			for (int j = 0; j < remaining_width; j++)
			{
				Disp[Disp_index] = LETTER_SPACE_SMALL.data[0];
				Disp_index++;
			}
			remaining_width = PIXEL_WIDTH;
		}

		for (int j = 0; j < table_letter.width; j++)
		{
			if (i != 0 && remaining_width < PIXEL_WIDTH) //fill in space between letters
			{
				Disp[Disp_index] = LETTER_SPACE_SMALL.data[0];
				Disp_index++;
			}

			Disp[Disp_index] = table_letter.data[j];
			Disp_index++;
			remaining_width--;
		}
	}

	for (int i = Disp_index; i < MAX_DISP_LENGTH; i++) //fill in rest of disp
	{
		Disp[i] = LETTER_SPACE_SMALL.data[0];
		Disp_index++;
	}

	Disp_Pic(Disp);
}

uint8_t Find_Char_Index(unsigned char lookup_letter, uint8_t small)
{
	if (small == 1)
	{
		for (int i = 0; i < 27; i++)
		{
			unsigned char table_letter = letter_small_table[i].ASCII;
			if (table_letter == lookup_letter)
			{
				return i;
			}
		}
		return 255; //not in table
	}
	return 255;
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
