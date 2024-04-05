/*
 * communication_utilities.c
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "communication_utilities.h"
#include "base_utilities.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/
#define MOTOR_ADDRESS '0'

/* Private macro -------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/


/* Private user code ---------------------------------------------------------*/
/*uint16_t Verify_Test()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','D','S', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive_Test(rx_buff); //DS2500\r
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[2], size - 3  );
	subbuff[size - 3] = '\0';

	return Array_To_Int_Test(subbuff, size - 2);
} */

void Analog_Mode()
{
	char buff[]={'@', MOTOR_ADDRESS, 'A', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Digital_Mode()
{
	char buff[]={'@', MOTOR_ADDRESS, 'D', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Set_To_Factory_Default()
{
	char buff[]={'@', MOTOR_ADDRESS, 'F', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Set_Inital_Value(uint8_t iv)
{
	if (iv < 0)
	{
		iv = 0;
	}
	else if (iv > 100)
	{
		iv = 100;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%cIV%c\r", MOTOR_ADDRESS, iv);
	Motor_Transmit(buff, size);
}

void Set_Gain_Value(uint8_t gv)
{
	if (gv < 0)
	{
		gv = 0;
	}
	else if (gv > 50)
	{
		gv = 50;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%cKG%c\r", MOTOR_ADDRESS, gv);
	Motor_Transmit(buff, size);
}

void Set_Integrator_Constant(uint8_t ic)
{
	if (ic < 0)
	{
		ic = 0;
	}
	else if (ic > 10)
	{
		ic = 10;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%cKI%c\r", MOTOR_ADDRESS, ic);
	Motor_Transmit(buff, size);
}

void Set_Proportional_Constant(uint8_t pc)
{
	if (pc < 0)
	{
		pc = 0;
	}
	else if (pc > 100)
	{
		pc = 100;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%cKP%c\r", MOTOR_ADDRESS, pc);
	Motor_Transmit(buff, size);
}

void Set_Digital_Speed(uint16_t ds)
{
	if (ds < 100)
	{
		ds = 100;
	}
	else if (ds > 9999)
	{
		ds = 9999;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%cM%d\r", MOTOR_ADDRESS, ds);
	Motor_Transmit(buff, size);
}

void Start_Motor()
{
	char buff[]={'@', MOTOR_ADDRESS, 'S', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

char Verify_Direction()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V', 'D', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //DS2500\r
	if (size == 0)
	{
		return '0';
	}

	return rx_buff[1]; //+ for CW, - for CCW
}

uint16_t Verify_Digital_Speed()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','D','S', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //DS2500\r
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[2], size - 3  );
	subbuff[size - 3] = '\0';

	return Array_To_Int(subbuff);
}

uint8_t Verify_Inital_Value()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','I','V', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //IV100\r
	if (size == 0)
	{
		return 0;
	}

	char subbuff[4];
	memcpy( subbuff, &rx_buff[2], size - 3 );
	subbuff[size - 3] = '\0';

	return Array_To_Int(subbuff);
}

uint8_t Verify_Gain_Value()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','K','G', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //KG50
	if (size == 0)
	{
		return 0;
	}

	char subbuff[3];
	memcpy( subbuff, &rx_buff[2], size - 3 );
	subbuff[size - 3] = '\0';

	return Array_To_Int(subbuff);
}

uint8_t Verify_Integrator_Constant()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','K','I', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //KI10
	if (size == 0)
	{
		return 0;
	}

	char subbuff[3];
	memcpy( subbuff, &rx_buff[2], size - 3 );
	subbuff[size - 3] = '\0';

	return Array_To_Int(subbuff);
}

uint8_t Verify_Proportional_Constant()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','K','P', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //KP100
	if (size == 0)
	{
		return 0;
	}

	char subbuff[4];
	memcpy( subbuff, &rx_buff[2], size - 3 );
	subbuff[size - 3] = '\0';

	return Array_To_Int(subbuff);
}

uint16_t Verify_Current_Speed()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','M', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //M9999
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[1], size - 2 );
	subbuff[size - 2] = '\0';

	return Array_To_Int(subbuff);
}

char Verify_Mode()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','M','O', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff);
	if (size == 0)
	{
		return '0';
	}

	return rx_buff[2]; //MOA analog, MOD digital
}

uint16_t Verify_Analog_Speed_Min()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','N', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //N9999
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[1], size - 2 );
	subbuff[size - 2] = '\0';

	return Array_To_Int(subbuff);
}

uint16_t Verify_Analog_Speed_Max()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','X', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //X9999
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[1], size - 2 );
	subbuff[size - 2] = '\0';

	return Array_To_Int(subbuff);
}

uint8_t Verify_Numer_Of_Poles()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','P', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //P12
	if (size == 0)
	{
		return 0;
	}

	char subbuff[3];
	memcpy( subbuff, &rx_buff[1], size - 2 );
	subbuff[size - 2] = '\0';

	return Array_To_Int(subbuff);
}

uint16_t Verify_Set_Speed()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','S', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //S9999
	if (size == 0)
	{
		return 0;
	}

	char subbuff[5];
	memcpy( subbuff, &rx_buff[1], size - 2 );
	subbuff[size - 2] = '\0';

	return Array_To_Int(subbuff);
}

char Verify_Brake_Setting()
{
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','S','T', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff);
	if (size == 0)
	{
		return '0';
	}

	return rx_buff[2]; //"ST." brake, "ST," coast
}

void Verify_All_Parameters()
{
	//prints out all set parameters, V*
	char tx_buff[]={'@', MOTOR_ADDRESS, 'V','*', '\r'};
	Motor_Transmit(tx_buff, sizeof(tx_buff));
	return;
}

void Coast()
{
	char buff[]={'@', MOTOR_ADDRESS, ',', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Hard_Brake()
{
	char buff[]={'@', MOTOR_ADDRESS, '.', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Set_Clockwise_Direction()
{
	char buff[]={'@', MOTOR_ADDRESS, '+', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

void Set_Counterclockwise_Direction()
{
	char buff[]={'@', MOTOR_ADDRESS, '-', '\r'};
	Motor_Transmit(buff, sizeof(buff));
}

uint8_t Verify_Address()
{
	char buff[]={'@','%', '\r'};
	Motor_Transmit(buff, sizeof(buff));

	uint8_t rx_buff[10];
	uint8_t size = Motor_Receive(rx_buff); //99
	if (size == 0)
	{
		return 0;
	}

	return Array_To_Int((char*)rx_buff);
}

void Set_Address(uint8_t a)
{
	if (a < 0)
	{
		a = 0;
	}
	else if (a > 99)
	{
		a = 99;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@~%c\r", a);
	Motor_Transmit(buff, size);
}

void Set_Analog_Speed_Min(uint16_t sm)
{
	if (sm < 100)
	{
		sm = 100;
	}
	else if (sm > 9999)
	{
		sm = 9999;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%c[%d\r", MOTOR_ADDRESS, sm);
	Motor_Transmit(buff, size);
}

void Set_Analog_Speed_Max(uint16_t sm)
{
	if (sm < 100)
	{
		sm = 100;
	}
	else if (sm > 9999)
	{
		sm = 9999;
	}

	char buff[50];
	uint8_t size = sprintf(buff, "@%c]%d\r", MOTOR_ADDRESS, sm);
	Motor_Transmit(buff, size);
}
