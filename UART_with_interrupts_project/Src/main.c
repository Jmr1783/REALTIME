/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "LED.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "LED.h"
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
RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "string.h"

#define MOV 0x20
#define WAIT 0x40
#define LOOP 0x80
#define END_LOOP 0xA0
#define RECIPE_END 0x00
#define POSITION_0 20
#define POSITION_1 17
#define POSITION_2 14
#define POSITION_3 11
#define POSITION_4 8
#define POSITION_5 4

uint8_t rx_buffer[20];  // Shared buffer between foreground and UART RX
uint8_t rx_byte;        // the currently received byte
uint8_t rx_index = 0;   // pointer into the rx_buffer
SemaphoreHandle_t  transmit_mutex;  // protects UART transmitter resource
SemaphoreHandle_t  receive_mutex;   // protects UART receiveer resource

enum status
{
	status_running,
	status_paused,
	status_command_error,
	status_nested_error,
	status_recipe_ended
};

enum events
{
	user_paused,
	user_continued,
	user_entered_right,
	user_entered_left,
	user_no_op,
	user_start,
	recipe_ended
};

uint8_t recipe0[] = {MOV | 0, MOV | 5, MOV | 0, MOV | 3, LOOP | 0, MOV | 1, MOV | 4, END_LOOP, MOV | 0, MOV | 2, WAIT | 0, MOV | 3, WAIT | 0, MOV | 2, MOV | 3, WAIT | 31, WAIT | 31, WAIT | 31, MOV | 4, RECIPE_END};
uint8_t recipe1[] = {MOV | 0, MOV | 1, MOV | 2, MOV | 3, MOV | 4, MOV | 5, MOV |4, MOV | 3, MOV | 2, MOV |1, RECIPE_END};

struct servo_stats
{
	enum status current_servo_status;
	enum events servo_events;
	uint32_t wait_count;
	uint8_t  op_code, range, recipe_command, servo_position, nested_check, loop_count, loop_start;
	
}servo[2];

TaskHandle_t servo1, servo2;

void task_init (void);
void vconsole_controller(void* argument);
void vPrintString(char* message);
void move_servo(TaskHandle_t servo_select, uint32_t position);
void check_status_leds(TaskHandle_t servo_select);

/*
 * prints the string, blocks if UART busy, thus safe from multiple threads
 */
void vPrintString(char *message) {
  xSemaphoreTake(transmit_mutex, ~0);         // Wait forever until the USART is free, then take mutex
  HAL_UART_Transmit_IT(&huart2, (uint8_t *)message, strlen(message));
  xSemaphoreGive(transmit_mutex);             // Give up mutex after printing
}

void process_user_commands(uint8_t aRxByte,TaskHandle_t servo_select){
	switch(aRxByte){
		case 'p':
		case 'P':
		{
			if (servo->current_servo_status == recipe_ended)
			{
				servo->servo_events = recipe_ended;
				servo->current_servo_status = status_paused;
			}
			else if (servo->current_servo_status  == status_command_error)
				servo->servo_events = user_no_op;
			else
			{
				servo->servo_events = user_paused;
				servo->current_servo_status = status_paused;
			}
			break;
		}
		case 'c':
		case 'C':
		{
			if (servo->current_servo_status == recipe_ended)
				servo->servo_events = recipe_ended;
			else if (servo->current_servo_status == status_command_error)
				servo->servo_events = user_no_op;
			else
				servo->servo_events = user_continued;
			break;
		}
		case 'r':
		case 'R':
		{
			if (servo->current_servo_status != status_paused)
				servo->servo_events = user_no_op;
			else if (servo->servo_position > 0)
				servo->servo_events = user_entered_right;
			else
				servo->servo_events = user_no_op;
			break;
		}
		case 'l':
		case 'L':
		{
			if (servo->current_servo_status != status_paused)
				servo->servo_events = user_no_op;
			else if (servo->servo_position < 5)
				servo->servo_events = user_entered_left;
			else
				servo->servo_events = user_no_op;
			break;
		}
		case 'n':
		case 'N':
		{
			servo->servo_events = user_no_op;
			break;
		}
		case 'b':
		case 'B':
		{
			servo->servo_events = user_start;
			break;
		}
		case 'x':
		case 'X':
		{
			break;
		}
		default:
		{
			return;
		}
	}
}

/*
 * overrides _weak HAL receiver callback, called when byte received
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xTaskWoken = pdFALSE;
  static BaseType_t i_have_receive_mutex = pdFALSE;
  if(huart->Instance == USART2) {

    // if received byte is a newline, give the received buffer to the forground
    if(rx_byte == '\r') {
	   xSemaphoreGiveFromISR(receive_mutex, &xTaskWoken);
      i_have_receive_mutex = pdFALSE;     // We don't have the mutex anymore
      rx_index = 0;                       // Next time around, queue data from start of buffer
    }
    
    // buffer all characters
    else {
      // acquire receive_mutex once
      if(!i_have_receive_mutex) {
        xSemaphoreTakeFromISR(receive_mutex,  &xTaskWoken);
        i_have_receive_mutex = pdTRUE;    // don't need to ask to Take again
      }
      
      // buffer all other characters
      rx_buffer[rx_index++] = rx_byte;    // buffer the byte
      rx_buffer[rx_index] = 0;            // keep string NULL terminated
      if(rx_index >= sizeof(rx_buffer))
        rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);  // one time, kick off receive interrupt (repeated from within callback)
  }
}

void vservo_task(void *argument)
{
	while(1)
	{
		check_status_leds(xTaskGetCurrentTaskHandle());
		vTaskDelay(pdMS_TO_TICKS(servo->wait_count)); //Need to convert wait count to MS
		servo->wait_count = 0;
	
		while((servo->current_servo_status == status_recipe_ended) || (servo->current_servo_status == status_paused))
		{
			continue;
		}
		
		if(xTaskGetCurrentTaskHandle() == servo1)
		{
			servo->op_code = recipe0[1] & 0xE0; //Add recipe indexing
			servo->range   = recipe0[1] & 0x1F;
		}
		else if(xTaskGetCurrentTaskHandle() == servo2)
		{
			servo->op_code = recipe1[1] & 0xE0; //Add recipe indexing
			servo->range   = recipe1[1] & 0x1F;
		}
		
		switch(servo->op_code)
		{
			case(MOV):
			{
				move_servo(xTaskGetCurrentTaskHandle(), servo->range);
				servo->current_servo_status = status_running;
				break;
			}
			case(WAIT):
			{
				servo->wait_count = servo->range;
				servo->current_servo_status = status_running;
				break;
			}
			case(LOOP):
			{
				if(servo->nested_check)
				{
					servo->current_servo_status = status_nested_error;
					break;
				}
				servo->loop_count = servo->range;
				if (xTaskGetCurrentTaskHandle() == servo1)
					servo->loop_start = recipe0[1];  //After adding recipe indexing this should point to the current recipe index
				else if (xTaskGetCurrentTaskHandle() == servo2)
					servo->loop_start = recipe1[1];
				else
				{
					servo->current_servo_status = status_command_error;
					break;
				}
				servo->nested_check = 1;
				servo->current_servo_status = status_running;
				break;
			}
			case(END_LOOP):
			{
				if(servo->loop_count)
				{
					servo->loop_count--;
					//servo recipe index is reset to servo->loop_start
					servo->current_servo_status = status_running;
				}
				else
				{
					servo->loop_start = NULL;
					servo->nested_check = 0;
					servo->current_servo_status = status_running;
				}
				break;
			}
			case(RECIPE_END):
			{
				servo->current_servo_status = status_recipe_ended;
				break;
			}
			default:
			{
				servo->current_servo_status = status_command_error;
				break;
			}
		}//switch
	}
}

//Completely initialize TIM2 CH1 and CH2 for PWM output
void timer2_pwm_init(void)
{
	//Enable clock to the GPIO pins that the PWM will output on
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	//Clear GPIOA MODER register
	GPIOA->MODER &= ~(0xFFFFFFFF);
	//Set PA0 and PA1 to alternate function
	GPIOA->MODER |= 10; //PA0
	GPIOA->MODER |= 10<<2; //PA1
	//Set PA0 and PA1 to TIM2 CH1 and CH2 respectively
	GPIOA->AFR[0] |= 1; //PA0 set to TIM2 CH1
	GPIOA->AFR[0] |= 1<<4; //PA1 set to TIM2 CH2
	//Enable clock to TIM2
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
	//Set TIM2 prescaler to 8000.  80MHz / 8000 = 20ms period
	TIM2->PSC = 8000;
	TIM2->CCMR1 &= ~(0x303);
	//Enable preload to CH1 and CH2
	TIM2->CCMR1 |= 1<<3; //CH1 preload
	TIM2->CCMR1 |= 1<<11; //CH2 preload
	//Set PWM mode 1 for both channels: CH1 and CH2
	TIM2->CCMR1 |= 110<<4; //PWM mode 1 CH1
	TIM2->CCMR1 |= 110<<12; //PWM mode 1 CH2
	TIM2->CCMR1 &= ~(1<<10);
	TIM2->CCMR1 &= ~(11<<8);
	//Set the auto reload preload functionality
	TIM2->CR1 |= 1<<7;
	//Enable compare capture output on CH1 and CH2
	TIM2->CCER |= 1; //CH1 compare capture output
	TIM2->CCER |= 1<<4; //CH2 compare capture output
	//Set auto reload register to 200
	TIM2->ARR = 200;
	TIM2->CCR1 = 50; //Default CH1 to 25% duty cycle
	TIM2->CCR2 = 50; //Default CH2 to 25% duty cycle
	//Set update event for prescaler
	TIM2->EGR |= TIM_EGR_UG;
	//Clear extraneous update flag
	TIM2->SR &= ~TIM_SR_UIF;
	//Enable TIM2 to run indefinitely.
	TIM2->CR1 = 0x1;
	
	return;
}


void timer3_init(void)
{
    /*
      Use update event to check for overflow
      Set in Upcounting mode
      Counts from 0 to 1000
      100us period * 1000 counts = 100ms
    */

    //Enable clock to TIM3
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    //Set TIM3 prescaler to 8000. 1/ fCK_PSC / (PSC[15:0] + 1) = 1/(80MHz / (7999 + 1) ) = 100us period
    TIM3->PSC = 7999;
    //Set the auto reload preload functionality
    TIM3->CR1 |= 1<<7;
    //Set auto reload register to 1000 for 100ms
    TIM3->ARR = 5000;
    //Set update event for prescaler
    TIM3->EGR |= TIM_EGR_UG;
    //Clear extraneous update flag
    TIM3->SR &= ~TIM_SR_UIF;
    //Enable TIM3 to run indefinitely.
    TIM3->CR1 = 0x1;

    return;
}

void move_servo(TaskHandle_t servo_select, uint32_t position)
{
	uint32_t pwm_position;
	
	//Assign pwm_positon based on command arguement which will hold proper CCR regsiter value for pwm
	switch(position)
	{
		case(0):
		{
			pwm_position = POSITION_0;
			break;
		}
		case(1):
		{
			pwm_position = POSITION_1;
			break;
		}
		case(2):
		{
			pwm_position = POSITION_2;
			break;
		}
		case(3):
		{
			pwm_position = POSITION_3;
			break;
		}
		case(4):
		{
			pwm_position = POSITION_4;
			break;
		}
		case(5):
		{
			pwm_position = POSITION_5;
			break;
		}
	}
	
	//Assign the value for PWM to the CCR register depending on which servo we are operation on
	if (position > 31)
		servo->current_servo_status = status_command_error;
	else if (servo_select == servo1)
	{
		TIM2->CCR1 = pwm_position;
		servo->servo_position = position;
	}
	else if (servo_select == servo2)
	{
		TIM2->CCR2 = pwm_position;
		servo->servo_position = position;
	}
	
	return;
}


void check_status_leds(TaskHandle_t servo_select)
{
	if(servo_select == servo1)
	{
		switch(servo->current_servo_status)
		{
			case(status_running):
			{
				Green_LED_On();
				Red_LED_Off();
				break;
			}
			case(status_paused):
			{
				Green_LED_Off();
				Red_LED_Off();
				break;
			}
			case(status_command_error):
			{
				Green_LED_Off();
				Red_LED_On();
				break;
			}
			case(status_nested_error):
			{
				Green_LED_On();
				Red_LED_On();
				break;
			}
			default:
			{
				Green_LED_On();
				Red_LED_On();
				break;
			}
		}
	}
	
	return;
}



void vconsole_controller(void *argument){

   while(1){
      // check to see if something has arrived over UART (e.g. a user command)
       if(rx_buffer[0] && (pdTRUE == xSemaphoreTake(receive_mutex, 0))) {  // if we Take mutex, ISR stops receiving chars
         process_user_commands(rx_buffer[0],servo1);
		   process_user_commands(rx_buffer[1],servo2);
         vPrintString((char *)rx_buffer);   // something arrived, stop receiving chars, print what we have.
         rx_buffer[0] = 0;  // flag ourself to NOT try to Take mutex until after ISR has received a new char (and owns mutex)
		   xSemaphoreGive(receive_mutex);
       } 

   }
}



/*
 * Initializes mutexes, interrupts, etc.  Creates tasks.
 */

void task_init (void) {
  transmit_mutex = xSemaphoreCreateMutex();     // create mutex to protect UART transmitter resource
  receive_mutex = xSemaphoreCreateMutex();      // create mutex to protect UART receiver resource
  xTaskCreate(vservo_task, "Servo1", 256, &servo[0], osPriorityNormal, servo1);
  xTaskCreate(vservo_task, "Servo2", 256, &servo[1], osPriorityNormal, servo2);
  xTaskCreate(vconsole_controller, "Console", 256, NULL, osPriorityNormal, NULL);
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);    // one time, kick off receive interrupt (repeated from within rx callback)

}

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
  MX_USART2_UART_Init();
  MX_RNG_Init();
  LED_Init();
  timer3_init();
  timer2_pwm_init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  task_init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV4;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
