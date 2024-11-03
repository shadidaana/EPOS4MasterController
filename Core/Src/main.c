/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
/*														    {Index } {Sub}										*/
uint8_t SET_PROFILE_VELOCITY_MODE_MESSAGE[8]=		{0x22, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};
uint8_t SET_PROFILE_POSITION_MODE_MESSAGE[8]=		{0x22, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};
uint8_t CONTROL_WORD_SHUTDOWN_MESSAGE[8]=			{0x22, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};
uint8_t CONTROL_WORD_SWITCH_ON_ENABLE_MESSAGE[8]=	{0x22, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};
uint8_t SET_RPM_MESSAGE[8]=							{0x22, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t STOP_WITHOUT_BRAKE_MESSAGE[8]= 				{0x22, 0x40, 0x60, 0x00, 0x0f, 0x01, 0x00, 0x00};
uint8_t STOP_WITH_BRAKE_MESSAGE[8]= 				{0x22, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};

int8_t start_motors =0;
uint8_t motorNodeID = 0x01;
MOTOR_STATE motors_state;
uint32_t speed;
CONTROL_MODE mode;
CAN_FilterTypeDef sFilterConfig;
uint32_t rsvFlag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int8_t init_motor(uint8_t nodeID);
int8_t init_motor_Position(uint8_t nodeID);
int8_t Set_Motor_Speed(uint8_t nodeID, uint32_t speed);
int8_t Set_Motor_Position(uint8_t nodeID, uint32_t position);
void send_can_message(uint32_t dlc, uint32_t stdId, uint8_t* data);
int8_t stop_motor_with_brake(uint8_t nodeID);
int8_t stop_motor_without_brake(uint8_t nodeID);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x000;
  sFilterConfig.FilterIdLow = 0x000;
  sFilterConfig.FilterMaskIdHigh = 0x000;
  sFilterConfig.FilterMaskIdLow = 0x000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
  	Error_Handler();
  }

  HAL_Delay(50);
  if(HAL_CAN_Start(&hcan1)!= HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    Error_Handler();
  }

  int8_t res;
  /*For debugging*/
  start_motors=1;
  mode= CLOCKWISE;
  speed =2000 ; /*0 - 9340*/
  uint32_t position= 6000;
  uint8_t status;
  // 3. Send few frames to allow EPOS4 to synchronize to the CAN bit rate
  uint8_t sync_message[1] = {0x00};  // Empty SYNC message
  for (int i = 0; i < 3; i++) {  // Sending 3 SYNC frames
      send_can_message(1, SYNC_COB_ID, sync_message);
      HAL_Delay(2);

  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t speed_neg = ~speed + 1;
//	  while(rsvFlag==0){
//
//	  }
		if (start_motors == 1)
		{
			send_can_message(8, MOTOR_HEART_BEAT_CAN_ID , SET_PROFILE_VELOCITY_MODE_MESSAGE);
			HAL_Delay(2);
			motors_state= HEART_BEAT;
			res = init_motor(motorNodeID);
//			init_motor_Position(motorNodeID);
			start_motors = 0;
		}
		  HAL_Delay(1000);
		if(mode == JOYSTICK){
		}
		else if (mode == CLOCKWISE)
		{
			res = Set_Motor_Speed(motorNodeID,speed);
		}
		else if (mode == COUNTERCLOCKWISE)
		{
			res = Set_Motor_Speed(motorNodeID, speed_neg);
		}

		else if (mode == STOP)
		{
			res = stop_motor_with_brake(motorNodeID);
		}
		else if (mode == PAUSE)
		{
			res = stop_motor_without_brake(motorNodeID);

		}
		else if (mode == POSITION){
			res = Set_Motor_Position(motorNodeID,position);
		}
		else if (mode != NONE)
		{
			uint8_t d[2] = {1, 1};
			send_can_message(2, 0x9F, d);
			mode = NONE;
		}

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 8;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN1_RX0_IRQHandler(void)
{
	/* USER CODE BEGIN CAN1_RX0_IRQn 0 */

	/* USER CODE END CAN1_RX0_IRQn 0 */
	HAL_CAN_IRQHandler(&hcan1);
	/* USER CODE BEGIN CAN1_RX0_IRQn 1 */

	CAN_RxHeaderTypeDef RxHeader;
	uint8_t RxData[8];
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
	rsvFlag=1;
	if (RxHeader.StdId == 0x0D9)
	{
			HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_15);
	}

	if (RxHeader.StdId == MOTOR_RESPONSE_CAN_ID + motorNodeID)
	{
		if (motors_state == SET_RPM_STATE || motors_state == SET_POSITION)
		{
			motors_state = CONTROL_WORD_SWITCH_ON_ENABLE_STATE;
		}
		else if (motors_state == CONTROL_WORD_SWITCH_ON_ENABLE_STATE)
		{
			motors_state = INITIALIZED;
		}
		else if (motors_state == STOP_STATE)
		{
			motors_state = INITIALIZED;
		}
		else if (motors_state == PAUSE_STATE)
		{
			motors_state = INITIALIZED;
		}
		else if (motors_state == SET_PROFILE_VELOCITY_STATE)
		{
			motors_state = CONTROL_WORD_SHUTDOWN_STATE;
		}
		else if (motors_state == CONTROL_WORD_SHUTDOWN_STATE)
		{
			motors_state = CONTROL_WORD_SWITCH_ON_ENABLE_STATE;
		}
		return;
	}

	else if (RxHeader.StdId == MOVE_MOTORS_CAN_ID)
	{

	}

	else if (RxHeader.StdId == PAUSE_MOTORS_CAN_ID)
	{
		mode = PAUSE;
		speed = 0;
		motors_state = PAUSE_STATE;
	}
	else if (RxHeader.StdId == STOP_MOTORS_CAN_ID)
	{
		mode = STOP;
		speed = 0;
		motors_state = STOP_STATE;
	}
	else if (RxHeader.StdId == START_MOTORS_CAN_ID) {
		speed = 0;
		mode = START;
		start_motors = 1;
		motors_state = HEART_BEAT;
	}

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

int8_t init_motor(uint8_t nodeID)
{
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	if(motors_state!= HEART_BEAT){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
//	HAL_Delay(1);
	motors_state= SET_PROFILE_VELOCITY_STATE;
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, SET_PROFILE_VELOCITY_MODE_MESSAGE);
	while(motors_state != CONTROL_WORD_SHUTDOWN_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SHUTDOWN_MESSAGE);
	while(motors_state != CONTROL_WORD_SWITCH_ON_ENABLE_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SWITCH_ON_ENABLE_MESSAGE);

	while(motors_state != INITIALIZED){
		HAL_Delay(1);
	}

	return 0;
}

int8_t init_motor_Position(uint8_t nodeID)
{
	//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
	if(motors_state!= HEART_BEAT){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
//	HAL_Delay(1);
	motors_state= SET_TARGET_POSITION_STATE;
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, SET_PROFILE_POSITION_MODE_MESSAGE);
	while(motors_state != CONTROL_WORD_SHUTDOWN_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SHUTDOWN_MESSAGE);
	while(motors_state != CONTROL_WORD_SWITCH_ON_ENABLE_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SWITCH_ON_ENABLE_MESSAGE);

	while(motors_state != INITIALIZED){
		HAL_Delay(1);
	}

	return 0;
}
int8_t Set_Motor_Speed(uint8_t nodeID, uint32_t speed)
{
	if(motors_state != INITIALIZED){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
	uint8_t SET_RPM_MESSAGE[8]={0x22, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

	SET_RPM_MESSAGE[7] = speed >> 24;
	SET_RPM_MESSAGE[6] = speed >> 16;
	SET_RPM_MESSAGE[5] = speed >> 8;
	SET_RPM_MESSAGE[4] = speed;
	motors_state = SET_RPM_STATE;
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, SET_RPM_MESSAGE);
	while (motors_state != CONTROL_WORD_SWITCH_ON_ENABLE_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SWITCH_ON_ENABLE_MESSAGE);

	while (motors_state != INITIALIZED){
		HAL_Delay(1);
	}
	return 0;
}

int8_t Set_Motor_Position(uint8_t nodeID, uint32_t position)
{
	if(motors_state != INITIALIZED){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
	uint8_t SET_POSISTION_MESSAGE[8]={0x22, 0x7A, 0x60, 0x00, 0xAE, 0x08, 0x00, 0x00};

	SET_POSISTION_MESSAGE[7] = position >> 24;
	SET_POSISTION_MESSAGE[6] = position >> 16;
	SET_POSISTION_MESSAGE[5] = position >> 8;
	SET_POSISTION_MESSAGE[4] = position;
	motors_state = SET_POSITION;
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, SET_POSISTION_MESSAGE);
	while (motors_state != CONTROL_WORD_SWITCH_ON_ENABLE_STATE){
		HAL_Delay(1);
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, CONTROL_WORD_SWITCH_ON_ENABLE_MESSAGE);

	while (motors_state != INITIALIZED){
		HAL_Delay(1);
	}
	return 0;
}

int8_t stop_motor_with_brake(uint8_t nodeID){
	if (motors_state != STOP_STATE){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, STOP_WITH_BRAKE_MESSAGE);
	while (motors_state != INITIALIZED){
		HAL_Delay(1);
	}
	return 0;
}

int8_t stop_motor_without_brake(uint8_t nodeID){
	if (motors_state != PAUSE_STATE){
		//HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		return -1;
	}
	send_can_message(8, MOTOR_COMMAND_CAN_ID + nodeID, STOP_WITHOUT_BRAKE_MESSAGE);
	while (motors_state != INITIALIZED){
		HAL_Delay(1);
	}
	return 0;
}


void send_can_message(uint32_t dlc, uint32_t stdId, uint8_t* data)
{
	 CAN_TxHeaderTypeDef TxHeader1;
	 TxHeader1.IDE = CAN_ID_STD;
	 TxHeader1.RTR = CAN_RTR_DATA;
	 TxHeader1.DLC = dlc;
	 TxHeader1.StdId = stdId;
	 uint32_t TxMailbox1;
	 HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, data, &TxMailbox1);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
