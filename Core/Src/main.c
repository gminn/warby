/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : This program implements the state machine for a robotic
  * 				          system.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef SystemState {
  Init_State,
  Transit_State,
  Object_Detect_Transit_State
};

typedef SystemEvent {
  Light_Detect_Event,
  Obstacle_Detect_Event
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FAST 110
#define MODERATE 70
#define SLOW 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t PC8591_ADDR = 0x48 << 1; // Use 8-bit address
int servo_pwm = 0;
int pwm_scale = 190;
int pwm_light1 = 0;
int pwm_light2 = 0;
int pwm_dist1 = 0;
int pwm_dist2 = 0;
int obj_right = 0;
int obj_left = 0;
double rebound_angle = 0;
int brightest_one = 0; // init_state var: record brightest light read
int brightest_two = 0; // init_state var: record brightest light read
uint8_t buf[20];
int light_vals[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t difference = 0;
uint8_t isFirstCaptured = 0;  // boolean: is the first value captured

/**
  * @brief Called when an interrupt is generated. Captures the "on" 
  * duration for the Echo PWM signal from the ultrasonic signal, 
  * which corresponds to the relative distance of the nearest object.
  * @modifies IC_Val1, ICVal2, difference, isFirstCaptured
  * @param htim (pointer to timer that triggered the interrupt)
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) { // interrupt source is TIM2
		if (!isFirstCaptured) { // first value not captured
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read first value
			isFirstCaptured = 1;  // set first captured as true

			// change polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}
		else {  // if the first already captured
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				difference = IC_Val2 - IC_Val1;
			}

			isFirstCaptured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
		} //else
	} //if
} //HAL_TIM_IC_CaptureCallback()

// EFFECTS: returns light intensity reading from photovoltaic cell
// on PC8591 module
int getLightIntensity() {
	HAL_StatusTypeDef ret;
	uint8_t buf[20];
	// Tell TMP102 that we want to read from the temperature register
	buf[0] = 0b01000000; // control byte
	ret = HAL_I2C_Master_Transmit(&hi2c1, PC8591_ADDR, buf, 1, HAL_MAX_DELAY);
	if ( ret != HAL_OK ) {
	  strcpy((char*)buf, "Error Tx\r\n");
	} else {

	  // Read 2 bytes from the temperature register
	  ret = HAL_I2C_Master_Receive(&hi2c1, PC8591_ADDR, buf, 2, HAL_MAX_DELAY);
	  if ( ret != HAL_OK ) {
		  strcpy((char*)buf, "Error Rx\r\n");
	  }
	}
	return buf[1]; // second read is more reliable
}

/**
  * @brief returns value of difference (global var) that is computed in 
  * HAL_TIM_IC_CaptureCallback()
  * @param None
  * @retval distance reading to nearest object detected by ultrasonic
  * sensor
  */
int getDist() {
	__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 5);
	return difference;
}

/**
  * @brief stops all motion
  * @param None
  * @retval None
  */
void brake() {
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);
}

/**
  * @brief pivots robot to the right (in place) for specified 
  * duration (timer)
  * @param timer (duration of pivot), use an int multiple of
  * pwm_scale
  * @retval None
  */
void turnR(uint32_t timer) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, FAST);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, FAST);
	HAL_Delay(timer);
	brake();
}

/**
  * @brief pivots robot to the left (in place) for specified 
  * duration (timer)
  * @param timer (duration of pivot), use an int multiple of
  * pwm_scale
  * @retval None
  */
void turnL(uint32_t timer) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, FAST);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, FAST);
	HAL_Delay(timer);
	brake();
}

/**
  * @brief moves robot forward at specified speed for 
  * specified duration (timer)
  * @param speed use macros SLOW, MODERATE, and FAST
  * @retval None
  */
void moveF(int speed) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
}

/**
  * @brief moves robot backwards at specified speed for 
  * specified duration (timer)
  * @param speed -- use macros SLOW, MODERATE, and FAST
  * @retval None
  */
void reverse(int speed) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 1);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, speed);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, speed);
}

/**
  * @brief manuvers around right side of object
  * @param None
  * @retval None
  */
void travelOnRight() {
  turnR(10*pwm_scale);
	moveF(FAST);
	turnL(5*pwm_scale);
}

/**
  * @brief manuvers around left side of object
  * @param None
  * @retval None
  */
void travelOnLeft() {
  turnL(10*pwm_scale);
  moveF(FAST);
  turnR(5*pwm_scale);
}

/**
  * @brief appropriately avoids object using boolean values 
  * indicating presence of objects on the right or left side of robot
  * @param objR -- true if object on the right
  * @param objL -- true if object on the left
  * @retval None
  */
// EFFECTS: 
void objectAvoid(bool objR, bool objL) {
	if(!objL) { // left side clear
		travelOnLeft();
	}
  else if(objR && objL) { // both sides occupied
		reverse(FAST);
    travelOnRight();
	}
	else { // right or both sides clear
		travelOnRight();
	}
}

SystemEvent ReadEvent(int *brightest) {
	SystemEvent event = Light_Detect_Event;
	uint8_t curr_light = 0;
	uint32_t curr_dist = 0;
//	double N = 21;
//	double alpha_0 = 8.57142857;
//	double num = 0; // numerator of rebound_angle
//	double denom = 0; // denominator of rebound_angle
//	int i = -N/2;
	obj_right = 0;
	obj_left = 0;
	int closest = 1;
	*brightest = 255;
	for(servo_pwm = 5; servo_pwm < 26; ++servo_pwm) {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, servo_pwm); // send servo pwm sig
		HAL_Delay(50);
		curr_light = getLightIntensity();
		if(curr_light < *brightest && event != Obstacle_Detect_Event) {
			pwm_light1 = servo_pwm;
			*brightest = curr_light;
		}
		curr_dist = getDist();
		if(curr_dist == closest && servo_pwm > 11 && servo_pwm < 19) {
			brake();
			closest = curr_dist;
			pwm_dist1 = servo_pwm;
			event = Obstacle_Detect_Event;
		}
//		num += (i*alpha_0*curr_dist);
//		denom += curr_dist;
//		++i;
	}
	sprintf((char*)buf, "LIGHT_PWM1 %u\r\n", (unsigned int) pwm_light1);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	*brightest = 255;
	for(servo_pwm = 24; servo_pwm > 5; --servo_pwm) {
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, servo_pwm); // send servo pwm sig
		HAL_Delay(50);
		curr_light = getLightIntensity();
		if(curr_light < *brightest && event != Obstacle_Detect_Event) {
			pwm_light2 = servo_pwm;
			*brightest = curr_light;
			event = Light_Detect_Event;
		}
		curr_dist = getDist();
		if(curr_dist == closest) {
			if(servo_pwm <= 11) {
				obj_right = 1;
			}
			else if(servo_pwm > 11 && servo_pwm < 19) {
				brake();
				closest = curr_dist;
				pwm_dist2 = servo_pwm;
				event = Obstacle_Detect_Event;
			}
			else if(servo_pwm >= 19) {
				obj_left = 1;
			}
		}
	}
	sprintf((char*)buf, "LIGHT_PWM2 %u\r\n", (unsigned int) pwm_light2);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//	rebound_angle = num/denom;
	return event;
}

SystemState LightDetectHandler()
{
	int average = (pwm_light1 + pwm_light2)/2;
	if(average > 15) {
		turnL((average - 15)*pwm_scale);
	}
	else {
		turnR((15 - average)*pwm_scale);
	}
	moveF(SLOW); // move indefinitely
    return Transit_State;
}

SystemState LightOutOfRangeHandler()
{
	turnL(20*pwm_scale); // do a 180
	ReadEvent(&brightest_two); // find brightest
	if(brightest_one < brightest_two) { // first read had our light sources
		turnL(20*pwm_scale); // turn back
	}
	brightest_one = 255; // reset vars
	brightest_two = 255;
	return Transit_State;
}

SystemState ObstacleDetectHandler()
{
	sprintf((char*)buf, "PWM_DIST1 %u\r\n", (unsigned int) pwm_dist1);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	sprintf((char*)buf, "PWM_DIST2 %u\r\n", (unsigned int) pwm_dist2);
	HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
	if(pwm_dist1 == 18 && pwm_dist2 == 12) { // large obj or two in front
		if(!obj_right && !obj_left) { // go right if both sides clear
			turnR(10*pwm_scale);
			moveF(1000, FAST);
			turnL(5*pwm_scale);
		}
		else if(obj_right && !obj_left) { // nothing to the left
			turnL(10*pwm_scale);
			moveF(1000, FAST);
			turnR(5*pwm_scale);
		}
		else if(!obj_right && obj_left) { // nothing to the right
			turnR(10*pwm_scale);
			moveF(1000, 110);
			turnL(5*pwm_scale);
		}
		else { // reverse if both sides occupied
			reverse(FAST);
			turnR(10*pwm_scale);
			moveF(700, 110);
			turnL(5*pwm_scale);
		}
	}
	else if(pwm_dist1 > 15) {
		if(!obj_right && !obj_left) { // go right if both sides clear
			turnR(7*pwm_scale);
			moveF(700, 110);
			turnL(5*pwm_scale);
		}
		else if(obj_right && !obj_left) { // nothing to the left
			turnL(7*pwm_scale);
			moveF(700, 110);
			turnR(5*pwm_scale);
		}
		else if(!obj_right && obj_left) { // nothing to the right
			turnR(7*pwm_scale);
			moveF(700, 110);
			turnL(5*pwm_scale);
		}
		else { // reverse if both sides occupied
			reverse(FAST);
			turnR(7*pwm_scale);
			moveF(700, 110);
			turnL(5*pwm_scale);
		}
	}
	else {
		if(!obj_right && !obj_left) { // go left if both sides clear
			turnL(7*pwm_scale);
			moveF(700, 110);
			turnR(5*pwm_scale);
		}
		else if(obj_right && !obj_left) { // nothing to the left
			turnL(7*pwm_scale);
			moveF(700, 110);
			turnR(5*pwm_scale);
		}
		else if(!obj_right && obj_left) { // nothing to the right
			turnR(7*pwm_scale);
			moveF(700, 110);
			turnL(5*pwm_scale);
		}
		else { // reverse if both sides occupied
			reverse(FAST);
			turnL(7*pwm_scale);
			moveF(700, 110);
			turnR(5*pwm_scale);
		}
	}

//	if(pwm_dist > 15) {
//		turnR(20*pwm_scale);
//	}
//	else {
//		turnL(20*pwm_scale);
//	}

//	double alpha_0 = 8.57142857;
//	double rebound = rebound_angle;
//	int angle = rebound_angle/alpha_0;
//	if(rebound_angle > 0) { // open space to left
//		turnL(angle*pwm_scale);
//	}
//	else { // open space to right
//		turnR(-1*angle*pwm_scale);
//	}
	moveF(SLOW);
    return Obstacle_Detect_Transit_State;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	SystemState NextState = Init_State;
	SystemEvent NewEvent;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  NewEvent = ReadEvent(&brightest_one);

	  switch(NextState) {
	  case Init_State:
		  if(NewEvent == Obstacle_Detect_Event) {
			  NextState = ObstacleDetectHandler();
		  }
		  else {
			  NextState = LightOutOfRangeHandler();
		  }
		  break;
	  case Transit_State:
		  if(NewEvent == Obstacle_Detect_Event) {
			  NextState = ObstacleDetectHandler();
		  }
		  else {
			  NextState = LightDetectHandler();
		  }
		  break;
	  case Obstacle_Detect_Transit_State:
		  if(NewEvent == Obstacle_Detect_Event) {
			  NextState = ObstacleDetectHandler();
		  }
//		  else if(NewEvent == Light_Out_Of_Range_Event) {
//			  NextState = LightOutOfRangeHandler();
//		  }
		  else {
			  NextState = LightDetectHandler();
		  }
		  break;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8399;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 42000;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 100;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
