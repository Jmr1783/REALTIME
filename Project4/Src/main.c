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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"
#include "rng.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define NUM_THREADS (3)
#define MAX_QUEUE_LENGTH (100)
#define NUM_STACK_DEPTH (256)
TaskHandle_t thread_handles [NUM_THREADS];
SemaphoreHandle_t HAL_mutex;
QueueHandle_t Q;

RNG_HandleTypeDef RNGen;

uint8_t bank_is_open = 0;
uint32_t current_queue_length, max_queue_length = 0;
uint32_t max_customer_wait_t=0, max_trans_time=0,max_time_customers_queue=0;
uint32_t total_time_customer_waiting =0, total_teller_idle_time=0;
uint32_t sim_sec = 0, sim_min = 0, sim_hr =  0, tick_cnt=0; //simulation global time
uint32_t customer_wait_t=0;
uint32_t idle_timer = 0;

struct teller_data{ //* Teller(int teller_num){
    uint32_t  customers_served;
   uint8_t  teller_status; //0 = idle and 1 = busy
   uint32_t  teller_ID;
    uint32_t total_time_with_customers;
   uint32_t avg_time_with_customers;
   uint32_t idle_time;
    uint32_t busy_time;
}Teller[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UART2_Init(void);
void UART2_GPIO_Init(void);
uint32_t random(uint32_t*,uint32_t, uint32_t);
void UART_Transmit(char*);
void debug(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void vApplicationIdleHook( void ) {
    idle_timer++;
}

void green_led_toggle()
{
	GPIOE->ODR ^= GPIO_ODR_ODR_8;
	for(int i = 0; i < 1000; i++){}
}

void red_led_toggle()
{
	GPIOB->ODR ^= GPIO_ODR_ODR_2;
	for(int i = 0; i < 1000; i++){}
}

void vBank( void *pvParameters ){
    uint32_t transaction_t = 0;
    uint32_t arrival_t = 0;
    uint32_t Queue_size = 0;

    
   while(1){
     Queue_size = uxQueueMessagesWaiting(Q);
     if( xSemaphoreTake (HAL_mutex, ( TickType_t ) 1000) == pdTRUE ){ // request access to data
     
     tick_cnt = xTaskGetTickCount(); // 1ms = 1 tick
     sim_sec = (uint32_t)(tick_cnt/1.666666)%60;    // 1.666 ticks = 1 sec sim
     sim_min = (tick_cnt/100)%60;  // 100 ticks = 1 ,min sim
     sim_hr =  (tick_cnt/6000)+8;  // 6000 ticks = 1 hr sim
      
      
      static char aTxByte[300];
      sprintf(aTxByte,
         "\fSystem Time: %d : %d : %d \n\r" 
         "Customers in line: %d customers\n\r" 
         "Teller 1 status: %d \n\r" 
         "Teller 2 status: %d \n\r" 
         "Teller 3 status: %d \n\r"
         "Bank Open: %d \n\r\n\r"
           ,
         sim_hr,sim_min,sim_sec,Queue_size,
         Teller[0].teller_status,Teller[1].teller_status,Teller[2].teller_status, bank_is_open
      ); 
      UART_Transmit(aTxByte);
      
      
      // sim for 24hr time for simplicity
      if((sim_hr >= 9)&&(sim_hr <16)){// check if 9am, bank should open
        bank_is_open = 1; //open the bank
        random(&arrival_t,60000,240000); //wait random 1 to 4 min
        random(&transaction_t,30000,480000); //wait random 30sec to 8mins
        vTaskDelay(pdMS_TO_TICKS(arrival_t));
        xQueueSendToBack( Q, &transaction_t, 0);
        Queue_size = uxQueueMessagesWaiting(Q);
        if (Queue_size > max_queue_length)
            max_queue_length = Queue_size;
        
      }
      
      if(sim_hr >= 16){ // check if 4pm, bank should close
         bank_is_open = 0; // close bank
         if(Queue_size == 0){
            static char msg[20];
            sprintf(msg,"Bank is closed...\n\r"); 
            UART_Transmit(msg);
            
            
            // get totals and avgs
            uint32_t  total_customers_served = Teller[0].customers_served
                                             +Teller[1].customers_served
                                             +Teller[2].customers_served;
            uint32_t total_customer_transaction_t = Teller[0].total_time_with_customers
                                                  +Teller[1].total_time_with_customers
                                                  +Teller[2].total_time_with_customers;
            uint32_t avg_customer_transaction_t= total_customer_transaction_t/ total_customers_served;
            uint32_t avg_time_waiting_customers = total_time_customer_waiting / total_customers_served;
            uint32_t avg_time_teller_idle = (total_teller_idle_time) /3; // needs some sort of average **********
            
            //print metrics
            static char aTxByte[500]; 
            sprintf(aTxByte,
               "Total customers served: %d customers\n\r" 
               "Teller 1 served: %d customers\n\r" 
               "Teller 2 served: %d customers\n\r" 
               "Teller 3 served: %d customers\n\r" 
               "Customer spends on avg in queue: %d mins\n\r" 
               "Customer spends on avg with teller: %d mins\n\r"
               "Teller spends on avg with customer: %d mins\n\r"    
               "Max time Customer spent in queue: %d mins\n\r" 
               "Max time teller waited for customer: %d mins\n\r"
               "Max time spent on a transaction: %d mins\n\r"
               "Max recorded customer line size: %d mins\n\r",
               total_customers_served, 
               Teller[0].customers_served, 
               Teller[1].customers_served, 
               Teller[2].customers_served,
               avg_time_waiting_customers,  
               avg_customer_transaction_t,
               avg_time_teller_idle,
               max_time_customers_queue,
               max_customer_wait_t, 
               max_trans_time,
               max_queue_length     
            ); 
            UART_Transmit(aTxByte);  //send to console
            vTaskSuspendAll();// suspend all tasks currently running
         }
      }
      xSemaphoreGive(HAL_mutex); // release access
		}
		vTaskDelay(100);
   }
}


void vTeller( void *argument )
{
	 uint32_t transaction_t = 0;
   uint32_t arrival_t = 0;
   uint32_t Q_size= 0;
     
    
    struct teller_data* teller = (struct teller_data*)argument; //pointer to specific teller for each thread
    
    while(1){  
    Q_size = uxQueueMessagesWaiting(Q);
    if (xQueueReceive(Q, &transaction_t,0)== pdPASS){ // grab a customer and check if move was successful
            if( xSemaphoreTake(HAL_mutex, ( TickType_t ) 10 ) == pdTRUE ){ // request access to data
               
               //update global or in struct: the customer wait time and busy time and other metrics
               teller->teller_status = 1; // update teller status
               
               
               customer_wait_t = (sim_sec - arrival_t);  // How long the customer is waiting to be seen
               total_time_customer_waiting += customer_wait_t;  // Total over all customers
               
               if(customer_wait_t> max_customer_wait_t){// max customer wait time update
                  max_customer_wait_t = customer_wait_t;
               }
               
               
               if(transaction_t > max_trans_time){// max trans time update
                  max_trans_time = transaction_t;
               }
                     
               teller->total_time_with_customers += transaction_t; // total serve time update
               
               teller->customers_served++; // total # of customers served
               
               xSemaphoreGive(HAL_mutex); // release access
            }
            
            static char aTxByte[50];
            sprintf(aTxByte,"\fTransaction_t: %d \n\r\n\r", transaction_t); 
            UART_Transmit(aTxByte);
            
            vTaskDelay(pdMS_TO_TICKS(transaction_t)); //wait 30s - 8min, state is busy until done
            teller->teller_status = 0; // the teller is now in idle
         }
   }
}
/*
void bank_thread (void *argument){ // address of teller data struct sent in as argument
static char aTxByte[50];
sprintf(aTxByte,"\fBank Running... \n\r\n\r"); 
UART_Transmit(aTxByte);
	
		
	  uint32_t transaction_t = 0;
    uint32_t arrival_t = 0;
    uint32_t Queue_size = 0;

    
   while(1){
//     Queue_size = uxQueueMessagesWaiting(Q);
//     if( xSemaphoreTake (HAL_mutex, ( TickType_t ) 1000) == pdTRUE ){ // request access to data
//     
//     tick_cnt = xTaskGetTickCount(); // 1ms = 1 tick
//     sim_sec = (uint32_t)(tick_cnt/1.666666)%60;    // 1.666 ticks = 1 sec sim
//     sim_min = (tick_cnt/100)%60;  // 100 ticks = 1 ,min sim
//     sim_hr =  (tick_cnt/6000);  // 6000 ticks = 1 hr sim
//      
//      
//      static char aTxByte[300];
//      sprintf(aTxByte,
//         "\fSystem Time: %d : %d : %d \n\r" 
//         "Customers in line: %d customers\n\r" 
//         "Teller 1 status: %d \n\r" 
//         "Teller 2 status: %d \n\r" 
//         "Teller 3 status: %d \n\r"
//         "Bank Open: %d \n\r\n\r"
//           ,
//         sim_hr,sim_min,sim_sec,Queue_size,
//         tellers[0].teller_status,tellers[2].teller_status,tellers[3].teller_status, bank_is_open
//      ); 
//      UART_Transmit(aTxByte);
//      
//      
//      // sim for 24hr time for simplicity
//      if((sim_hr >= 9)&&(sim_hr <16)){// check if 9am, bank should open
//        bank_is_open = 1; //open the bank
//        random(&arrival_t,60000,240000); //wait random 1 to 4 min
//        random(&transaction_t,30000,480000); //wait random 30sec to 8mins
//        vTaskDelay(pdMS_TO_TICKS(arrival_t));
//        xQueueSendToBack( Q, &transaction_t, 0);
//        Queue_size = uxQueueMessagesWaiting(Q);
//        if (Queue_size > max_queue_length)
//            max_queue_length = Queue_size;
//        
//      }
//      
//      if(sim_hr >= 16){ // check if 4pm, bank should close
//         bank_is_open = 0; // close bank
//         if(Queue_size == 0){
//            static char msg[20];
//            sprintf(msg,"Bank is closed...\n\r"); 
//            UART_Transmit(msg);
//            
//            
//            // get totals and avgs
//            uint32_t  total_customers_served = Teller[0].customers_served
//                                             +Teller[1].customers_served
//                                             +Teller[2].customers_served;
//            uint32_t total_customer_transaction_t = Teller[0].total_time_with_customers
//                                                  +Teller[1].total_time_with_customers
//                                                  +Teller[2].total_time_with_customers;
//            uint32_t avg_customer_transaction_t= total_customer_transaction_t/ total_customers_served;
//            uint32_t avg_time_waiting_customers = total_time_customer_waiting / total_customers_served;
//            uint32_t avg_time_teller_idle = (total_teller_idle_time) /3; // needs some sort of average **********
//            
//            //print metrics
//            static char aTxByte[500]; 
//            sprintf(aTxByte,
//               "Total customers served: %d customers\n\r" 
//               "Teller 1 served: %d customers\n\r" 
//               "Teller 2 served: %d customers\n\r" 
//               "Teller 3 served: %d customers\n\r" 
//               "Customer spends on avg in queue: %d mins\n\r" 
//               "Customer spends on avg with teller: %d mins\n\r"
//               "Teller spends on avg with customer: %d mins\n\r"    
//               "Max time Customer spent in queue: %d mins\n\r" 
//               "Max time teller waited for customer: %d mins\n\r"
//               "Max time spent on a transaction: %d mins\n\r"
//               "Max recorded customer line size: %d mins\n\r",
//               total_customers_served, 
//               Teller[0].customers_served, 
//               Teller[1].customers_served, 
//               Teller[2].customers_served,
//               avg_time_waiting_customers,  
//               avg_customer_transaction_t,
//               avg_time_teller_idle,
//               max_time_customers_queue,
//               max_customer_wait_t, 
//               max_trans_time,
//               max_queue_length     
//            ); 
//            UART_Transmit(aTxByte);  //send to console
//            vTaskSuspendAll();// suspend all tasks currently running
//         }
//      }
//      xSemaphoreGive(HAL_mutex); // release access
//		}
		vTaskDelay(100);
   }
   
}
*/

/*
void teller_thread (void *argument){
static char aTxByte[50];
sprintf(aTxByte,"\fTeller Running... \n\r\n\r"); 
UART_Transmit(aTxByte);
   uint32_t transaction_t = 0;
   uint32_t arrival_t = 0;
   uint32_t Q_size= 0;
     
    
    struct teller_data* teller = (struct teller_data*)argument; //pointer to specific teller for each thread
    
    while(1){  
//    Q_size = uxQueueMessagesWaiting(Q);
//      ///if((Q_size != 0)){ //if queue is not size 0 
//         if (xQueueReceive(Q, &transaction_t,0)== pdPASS){ // grab a customer and check if move was successful
//            if( xSemaphoreTake(HAL_mutex, ( TickType_t ) 10 ) == pdTRUE ){ // request access to data
//               
//               //update global or in struct: the customer wait time and busy time and other metrics
//               teller->teller_status = 1; // update teller status
//               
//               
//               customer_wait_t = (sim_sec - arrival_t);  // How long the customer is waiting to be seen
//               total_time_customer_waiting += customer_wait_t;  // Total over all customers
//               
//               if(customer_wait_t> max_customer_wait_t){// max customer wait time update
//                  max_customer_wait_t = customer_wait_t;
//               }
//               
//               
//               if(transaction_t > max_trans_time){// max trans time update
//                  max_trans_time = transaction_t;
//               }
//                     
//               teller->total_time_with_customers += transaction_t; // total serve time update
//               
//               teller->customers_served++; // total # of customers served
//               
//               xSemaphoreGive(HAL_mutex); // release access
//            }
//            
//            static char aTxByte[50];
//            sprintf(aTxByte,"\fTransaction_t: %d \n\r\n\r", transaction_t); 
//            UART_Transmit(aTxByte);
//            
//            vTaskDelay(pdMS_TO_TICKS(transaction_t)); //wait 30s - 8min, state is busy until done
//            teller->teller_status = 0; // the teller is now in idle
//         }
		vTaskDelay(100);
   }
}
*/
void thread_init (void) {
   HAL_mutex = xSemaphoreCreateMutex ();
   Q = xQueueCreate(MAX_QUEUE_LENGTH, sizeof(uint32_t));
	 xTaskCreate (vBank,"Bank", NUM_STACK_DEPTH, NULL, 1, NULL);
	 xTaskCreate (vTeller,"Teller1", NUM_STACK_DEPTH, &Teller[0], 1, NULL);
	 xTaskCreate (vTeller,"Teller2", NUM_STACK_DEPTH, &Teller[1], 1, NULL);
	 xTaskCreate (vTeller,"Teller3", NUM_STACK_DEPTH, &Teller[2], 1, NULL);
} 
 

uint32_t random(uint32_t *pointer,uint32_t max, uint32_t min){
    return (HAL_RNG_GenerateRandomNumber( &RNGen, pointer) % (max+1 - min) + min);
}

void UART_Transmit(char* aTxByte){
    xSemaphoreTake (HAL_mutex, ( TickType_t ) 10);
    HAL_UART_Transmit(&huart2,(uint8_t*) aTxByte, strlen(aTxByte), 1000000);
    xSemaphoreGive (HAL_mutex);
}

void resource_display(){
    char aTxByte[100];
   int x = uxTaskGetNumberOfTasks();
   int y = xPortGetFreeHeapSize();
   int z = xPortGetMinimumEverFreeHeapSize();
    sprintf(aTxByte,
   "%d Running\n\r"
   "%d Bytes Free\n\r"
   "%d Min Bytes Free\n\r", x,y,z); 
   UART_Transmit(aTxByte);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  
  /* USER CODE BEGIN 2 */
    
    // TODO: Your code here

  thread_init();
  
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  HAL_Delay(100);
  //resource_display();
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure LSE Drive Capability 
    */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Enable MSI Auto calibration 
    */
  HAL_RCCEx_EnableMSIPLLMode();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */
void UART2_Init(void) {
        // Enable the clock of USART 1 & 2
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;  // Enable USART 2 clock        
    
    // Select the USART1 clock source
    // 00: PCLK selected as USART2 clock
    // 01: System clock (SYSCLK) selected as USART2 clock
    // 10: HSI16 clock selected as USART2 clock
    // 11: LSE clock selected as USART2 clock
    RCC->CCIPR &= ~RCC_CCIPR_USART2SEL;
    RCC->CCIPR |=  RCC_CCIPR_USART2SEL_0;
    
    UART2_GPIO_Init();
    //USART_Init(USART2);
    
    USART2->CR1 |= USART_CR1_RXNEIE;            // Received Data Ready to be Read Interrupt  
    NVIC_SetPriority(USART2_IRQn, 0);           // Set Priority to 1
    NVIC_EnableIRQ(USART2_IRQn);                    // Enable interrupt of USART peripheral
}

void UART2_GPIO_Init(void) {
    
    // Enable the peripheral clock of GPIO Port
    RCC->AHB2ENR |=   RCC_AHB2ENR_GPIODEN;
    
    // ********************** USART 2 *************************** 
    // PD5 = USART2_TX (AF7)
    // PD6 = USART2_RX (AF7)
    // Alternate function, High Speed, Push pull, Pull up
    // **********************************************************
    // Input(00), Output(01), AlterFunc(10), Analog(11)
    GPIOD->MODER   &= ~(0xF << (2*5));  // Clear bits
    GPIOD->MODER   |=   0xA << (2*5);           
    GPIOD->AFR[0]  |=   0x77<< (4*5);           
    // GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
    GPIOD->OSPEEDR |=   0xF<<(2*5);                         
    // GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
    GPIOD->PUPDR   &= ~(0xF<<(2*5));
    GPIOD->PUPDR   |=   0x5<<(2*5);                 
    // GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
    GPIOD->OTYPER  &=  ~(0x3<<5) ;          
}

/* StartDefaultTask function */


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
char err_msg[100];
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  sprintf(err_msg, "ERROR: %s(%d)\r\n", file, line);
    HAL_UART_Transmit(&huart2, (uint8_t *)err_msg, strlen(err_msg), 1000000);
    while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
