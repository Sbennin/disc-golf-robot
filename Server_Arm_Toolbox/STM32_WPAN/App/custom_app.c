/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* Server_Arm */
  uint8_t               State_c_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  Motor_State_t State_Status;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Server_Arm */
static void Custom_State_c_Update_Char(void);
static void Custom_State_c_Send_Notification(void);

/* USER CODE BEGIN PFP */
uint16_t Payload_To_Speed(uint8_t, uint8_t);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* Server_Arm */
    case CUSTOM_STM_GOAL_C_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_GOAL_C_READ_EVT */

      /* USER CODE END CUSTOM_STM_GOAL_C_READ_EVT */
      break;

    case CUSTOM_STM_GOAL_C_WRITE_NO_RESP_EVT:
      /* USER CODE BEGIN CUSTOM_STM_GOAL_C_WRITE_NO_RESP_EVT */
        APP_DBG_MSG("\r\n\r** CUSTOM_STM_GOAL_C_WRITE_NO_RESP_EVT \n");
        APP_DBG_MSG("\r\n\r** Write Data: 0x%02X %02X \n", pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
        uint16_t goal_speed = Payload_To_Speed(pNotification->DataTransfered.pPayload[0], pNotification->DataTransfered.pPayload[1]);
        if (goal_speed < 301)
        {
        	HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);
        }
        else if(goal_speed < 601)
        {
        	HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_RESET);
        }
        else
        {
        	HAL_GPIO_WritePin(Blue_Led_GPIO_Port, Blue_Led_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(Green_Led_GPIO_Port, Green_Led_Pin, GPIO_PIN_RESET);
        	HAL_GPIO_WritePin(Red_Led_GPIO_Port, Red_Led_Pin, GPIO_PIN_SET);
        }
      /* USER CODE END CUSTOM_STM_GOAL_C_WRITE_NO_RESP_EVT */
      break;

    case CUSTOM_STM_STATE_C_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_STATE_C_NOTIFY_ENABLED_EVT */
        APP_DBG_MSG("\r\n\r** CUSTOM_STM_STATE_C_NOTIFY_ENABLED_EVT \n");

        Custom_App_Context.State_c_Notification_Status = 1;
      /* USER CODE END CUSTOM_STM_STATE_C_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_STATE_C_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_STATE_C_NOTIFY_DISABLED_EVT */
        APP_DBG_MSG("\r\n\r** CUSTOM_STM_STATE_C_NOTIFY_DISABLED_EVT \n");

        Custom_App_Context.State_c_Notification_Status = 0;
      /* USER CODE END CUSTOM_STM_STATE_C_NOTIFY_DISABLED_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	  UTIL_SEQ_RegTask(1<< CFG_TASK_B1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Custom_State_c_Send_Notification);
	  UTIL_SEQ_RegTask(1<< CFG_TASK_B2_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Custom_State_c_Send_Notification);
	  UTIL_SEQ_RegTask(1<< CFG_TASK_B3_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, Custom_State_c_Send_Notification);

	  Custom_App_Context.State_c_Notification_Status = 0;
	  Custom_App_Context.State_Status = STOPPED;
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* Server_Arm */
void Custom_State_c_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN State_c_UC_1*/

  /* USER CODE END State_c_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_STATE_C, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN State_c_UC_Last*/

  /* USER CODE END State_c_UC_Last*/
  return;
}

void Custom_State_c_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN State_c_NS_1*/
  if(Custom_App_Context.State_c_Notification_Status)
  {
	  updateflag = 1;

	  if(Custom_App_Context.State_Status == STOPPED)
	  {
		  NotifyCharData[0] = 0x00; //Big Endian
		  NotifyCharData[1] = 0x00;
		  APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT MOTOR STOPPED \n");
		  APP_DBG_MSG(" \n\r");
	  }
	  else if (Custom_App_Context.State_Status == PENDING)
	  {
		  NotifyCharData[0] = 0x00;
		  NotifyCharData[1] = 0x01;
		  APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT MOTOR PENDING \n");
		  APP_DBG_MSG(" \n\r");
	  }
	  else if (Custom_App_Context.State_Status == DONE)
	  {
		  NotifyCharData[0] = 0x00;
		  NotifyCharData[1] = 0x02;
		  APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT MOTOR DONE \n");
		  APP_DBG_MSG(" \n\r");
	  }
	  else
	  {
		  NotifyCharData[0] = 0x00;
		  NotifyCharData[1] = 0x03;
		  APP_DBG_MSG("-- CUSTOM APPLICATION SERVER  : INFORM CLIENT MOTOR WRONG STATE \n");
		  APP_DBG_MSG(" \n\r");
	  }
  }
  else
  {
	  APP_DBG_MSG("-- CUSTOM APPLICATION : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n");
  }
  /* USER CODE END State_c_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_STATE_C, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN State_c_NS_Last*/

  /* USER CODE END State_c_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
void P2PS_APP_B1_Button_Action(void)
{
	Custom_App_Context.State_Status = STOPPED;
  UTIL_SEQ_SetTask(1<<CFG_TASK_B1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void P2PS_APP_B2_Button_Action(void)
{
	Custom_App_Context.State_Status = PENDING;
  UTIL_SEQ_SetTask(1<<CFG_TASK_B2_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void P2PS_APP_B3_Button_Action(void)
{
	Custom_App_Context.State_Status = DONE;
  UTIL_SEQ_SetTask(1<<CFG_TASK_B3_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

uint16_t Payload_To_Speed(uint8_t p0, uint8_t p1)
{
    uint16_t result = (p0 << 8) + p1;
    return result;
}
/* USER CODE END FD_LOCAL_FUNCTIONS*/
