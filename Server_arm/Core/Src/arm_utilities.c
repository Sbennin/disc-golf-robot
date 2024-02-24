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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
uint32_t state;
uint32_t state_changed;
encoder_instance motor_instance;
/* USER CODE END PV */


