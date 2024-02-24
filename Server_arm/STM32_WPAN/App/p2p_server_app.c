/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    p2p_server_app.c
  * @author  MCD Application Team
  * @brief   Peer to peer Server Application
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2021 STMicroelectronics.
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
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 typedef struct{
    uint8_t             Device_Led_Selection;
    uint8_t             Led1;
 }P2P_LedCharValue_t;

 typedef struct{
    uint8_t             Device_Button_Selection;
    uint8_t             ButtonStatus;
 }P2P_ButtonCharValue_t;

typedef struct
{
  uint8_t               Notification_Status; // used to check if P2P Server is enabled to Notify
  P2P_LedCharValue_t    LedControl;
  P2P_ButtonCharValue_t ButtonControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;
*/

typedef struct
{
  uint8_t               Notification_Status; // used to check if P2P Server is enabled to Notify
  P2P_GoalSpeed_t       GoalControl;
  P2P_MotorState_t      MotorStateControl;
  uint16_t              ConnectionHandle;
} P2P_Server_App_Context_t;

enum Motor_State {
  STOPPED,
  PENDING,
  DONE
};

typedef struct
{
	uint16_t GoalSpeed;
}P2P_GoalSpeed_t;

typedef struct
{
	Motor_State MotorState;
}P2P_MotorState_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**
 * START of Section BLE_APP_CONTEXT
 */

static P2P_Server_App_Context_t P2P_Server_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//static void P2PS_Send_Notification(void);
//static void P2PS_APP_LED_BUTTON_context_Init(void);
static void P2PS_APP_MOTOR_context_Init(void);
static void P2PS_Send_Notification_Stopped(void);
static void P2PS_Send_Notification_Pending(void);
static void P2PS_Send_Notification_Done(void);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_STM_App_Notification_1 */

/* USER CODE END P2PS_STM_App_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
    case P2PS_STM_BOOT_REQUEST_EVT:
      APP_DBG_MSG("-- P2P APPLICATION SERVER : BOOT REQUESTED\n");
      APP_DBG_MSG(" \n\r");

      *(uint32_t*)SRAM1_BASE = *(uint32_t*)pNotification->DataTransfered.pPayload;
      NVIC_SystemReset();
      break;
#endif
/* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

    case P2PS_STM__NOTIFY_ENABLED_EVT:
/* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 1;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION ENABLED\n"); 
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
      break;

    case P2PS_STM_NOTIFY_DISABLED_EVT:
/* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
      P2P_Server_App_Context.Notification_Status = 0;
      APP_DBG_MSG("-- P2P APPLICATION SERVER : NOTIFICATION DISABLED\n");
      APP_DBG_MSG(" \n\r");
/* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
      break;

    case P2PS_STM_WRITE_EVT:
/* USER CODE BEGIN P2PS_STM_WRITE_EVT */
    	//TODO get motor goal speed from client
      if(pNotification->DataTransfered.pPayload[0] == 0x00){ /* ALL Deviceselected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER  : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#if(P2P_SERVER1 != 0)
      if(pNotification->DataTransfered.pPayload[0] == 0x01){ /* end device 1 selected - may be necessary as LB Routeur informs all connection */
        if(pNotification->DataTransfered.pPayload[1] == 0x01)
        {
          BSP_LED_On(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 ON\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x01; /* LED1 ON */
        }
        if(pNotification->DataTransfered.pPayload[1] == 0x00)
        {
          BSP_LED_Off(LED_BLUE);
          APP_DBG_MSG("-- P2P APPLICATION SERVER 1 : LED1 OFF\n"); 
          APP_DBG_MSG(" \n\r");
          P2P_Server_App_Context.LedControl.Led1=0x00; /* LED1 OFF */
        }
      }
#endif
/* USER CODE END P2PS_STM_WRITE_EVT */
      break;

    default:
/* USER CODE BEGIN P2PS_STM_App_Notification_default */
      
/* USER CODE END P2PS_STM_App_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_STM_App_Notification_2 */

/* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification)
{
/* USER CODE BEGIN P2PS_APP_Notification_1 */
//TODO incoming data function
/* USER CODE END P2PS_APP_Notification_1 */
  switch(pNotification->P2P_Evt_Opcode)
  {
/* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

/* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT :
/* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
          
/* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

    case PEER_DISCON_HANDLE_EVT :
/* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
       //P2PS_APP_LED_BUTTON_context_Init();       //TODO
       P2PS_APP_MOTOR_context_Init();
/* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

    default:
/* USER CODE BEGIN P2PS_APP_Notification_default */

/* USER CODE END P2PS_APP_Notification_default */
      break;
  }
/* USER CODE BEGIN P2PS_APP_Notification_2 */

/* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void)
{
/* USER CODE BEGIN P2PS_APP_Init */
	//TODO registers send notification task to sequencer under ID
  //UTIL_SEQ_RegTask( 1<< CFG_TASK_SW1_BUTTON_PUSHED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification );
  UTIL_SEQ_RegTask( 1<< CFG_TASK_MOTOR_STOPPED_ID, UTIL_SEQ_RFU, P2PS_Send_Notification_Stopped);
  UTIL_SEQ_RegTask( 1<< CFG_TASK_MOTOR_PENDING_ID, UTIL_SEQ_RFU, P2PS_Send_Notification_Pending);
  UTIL_SEQ_RegTask( 1<< CFG_TASK_MOTOR_DONE_ID, UTIL_SEQ_RFU, P2PS_Send_Notification_Done);

  /**
   * Initialize LedButton Service
   */
  //TODO
  P2P_Server_App_Context.Notification_Status=0; 
  //P2PS_APP_LED_BUTTON_context_Init();
  P2PS_APP_MOTOR_context_Init();
/* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
/*void P2PS_APP_LED_BUTTON_context_Init(void){
  //TODO initializes peripheral state
  BSP_LED_Off(LED_BLUE);
  APP_DBG_MSG("LED BLUE OFF\n");
  
  //TODO initializes struct
  #if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.LedControl.Device_Led_Selection=0x01; // Device1
  P2P_Server_App_Context.LedControl.Led1=0x00; // led OFF
  P2P_Server_App_Context.ButtonControl.Device_Button_Selection=0x01;// Device1
  P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
#endif
}*/

void P2PS_APP_MOTOR_context_Init(void){
  //initializes peripheral state
  Stop_Motor();
  Set_CW();
  APP_DBG_MSG("MOTOR STOPPED\n");

  //initializes struct
  #if(P2P_SERVER1 != 0)
  P2P_Server_App_Context.GoalControl.GoalSpeed=0; //goal speed is 0
  P2P_Server_App_Context.MotorStateControl.MotorState=STOPPED;
#endif
}

void P2PS_APP_SW1_Button_Action(void) //run from button interrupt
{
	//TODO runs task from sequencer, this ID corresponds to send notification
  UTIL_SEQ_SetTask( 1<<CFG_TASK_SW1_BUTTON_PUSHED_ID, CFG_SCH_PRIO_0);

  return;
}

void Motor_Stopped_Complete(void)
{
	UTIL_SEQ_SetTask( 1<<CFG_TASK_MOTOR_STOPPED_ID, CFG_SCH_PRIO_0);

	return;
}

void Motor_Pending_Complete(void)
{
	UTIL_SEQ_SetTask( 1<<CFG_TASK_MOTOR_PENDING_ID, CFG_SCH_PRIO_0);

	return;
}

void Motor_Done_Complete(void)
{
	UTIL_SEQ_SetTask( 1<<CFG_TASK_MOTOR_DONE_ID, CFG_SCH_PRIO_0);

	return;
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/
/*void P2PS_Send_Notification(void) //run from sequencer
{
 //TODO updating struct
  if(P2P_Server_App_Context.ButtonControl.ButtonStatus == 0x00){
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x01;
  } else {
    P2P_Server_App_Context.ButtonControl.ButtonStatus=0x00;
  }
  
  //TODO sending notification of new button status
   if(P2P_Server_App_Context.Notification_Status){ 
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT BUTTON 1 PUSHED \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.MotorStateControl.MotorState);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n "); 
   }

  return;
} */

void P2PS_Send_Notification_Done(void) //run from sequencer
{
	P2P_Server_App_Context.MotorStateControl.MotorState = DONE;

  //TODO sending notification of new button status
   if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT MOTOR DONE \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.MotorStateControl.MotorState);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}

void P2PS_Send_Notification_Stopped(void) //run from sequencer
{
	P2P_Server_App_Context.MotorStateControl.MotorState = STOPPED;

  //TODO sending notification of new button status
   if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT MOTOR STOPPED \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.MotorStateControl.MotorState);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}

void P2PS_Send_Notification_Pending(void) //run from sequencer
{
	P2P_Server_App_Context.MotorStateControl.MotorState = PENDING;

  //TODO sending notification of new button status
   if(P2P_Server_App_Context.Notification_Status){
    APP_DBG_MSG("-- P2P APPLICATION SERVER  : INFORM CLIENT MOTOR PENDING \n ");
    APP_DBG_MSG(" \n\r");
    P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, (uint8_t *)&P2P_Server_App_Context.MotorStateControl.MotorState);
   } else {
    APP_DBG_MSG("-- P2P APPLICATION SERVER : CAN'T INFORM CLIENT -  NOTIFICATION DISABLED\n ");
   }

  return;
}


/* USER CODE END FD_LOCAL_FUNCTIONS*/
