/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define NUMBER_OF_MOTORS 4

#define MOTOR_HEART_BEAT_CAN_ID			0x700
#define MOTOR_COMMAND_CAN_ID				0x600
#define MOTOR_RESPONSE_CAN_ID				0x580

#define FRONT_RIGHT_MOTOR_ID 				0x02
#define FRONT_LEFT_MOTOR_ID 				0x04
#define BACK_LEFT_MOTOR_ID 					0x03
#define BACK_RIGHT_MOTOR_ID 				0x01

#define START_MOTORS_CAN_ID					0x090
#define MOVE_MOTORS_CAN_ID					0x091
#define PAUSE_MOTORS_CAN_ID					0x092
#define STOP_MOTORS_CAN_ID					0x093

#define SYNC_COB_ID         0x080  // COB-ID for SYNC message

typedef enum CONTROL_MODE_ENUM
{
	STOP,
	CLOCKWISE,
	COUNTERCLOCKWISE,
	JOYSTICK,
	NONE,
	PAUSE,
	POSITION,
	START
} CONTROL_MODE;
typedef enum MOTOR_STATE_ENUM
{
	HEART_BEAT,
	SET_PROFILE_VELOCITY_STATE,
	SET_TARGET_POSITION_STATE,
	CONTROL_WORD_SHUTDOWN_STATE,
	CONTROL_WORD_SWITCH_ON_ENABLE_STATE,
	SET_RPM_STATE,
	SET_POSITION,
	STOP_STATE,
	PAUSE_STATE,
	INITIALIZED,
	NONE_STATE
} MOTOR_STATE;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
