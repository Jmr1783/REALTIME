/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"
#include "../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"

#define GYRO_THRESHOLD_DETECTION (10000)    /* Tunnng parameters for gyroscope */

extern int servo2_position;
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;
/* Gyroscope variables */
float gyro_velocity[3] = {0};       // angular velocity
int32_t gyro_angle[3] = {0, 0, 0};  // angle

void gyro_task(void* argument);  

//****************************************************************************************************
/*
	FUNCTION THAT INITIALIZIES THE THREAD THAT RUNS THE GYRO
	INPUT PARAMETERS: void INPUT TYPE
	FUNCTION OUTPUT: void OUTPUT TYPE
*/
void gyro_task_init() {
  
  if(BSP_GYRO_Init() != HAL_OK)  {
    /* Initialization Error */
    Error_Handler();
  }
  if (pdPASS != xTaskCreate (gyro_task, "print", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

//****************************************************************************************************
/*
	FUNCTION THAT GATHERS DATA FROM GYRO AND UPDATES POSITION OF SERVO2
	INPUT PARAMETERS: void INPUT TYPE
	FUNCTION OUTPUT: void OUTPUT TYPE
*/
void gyro_task(void* argument) {
  while(1) {
     
    BSP_GYRO_GetXYZ(gyro_velocity);   // get raw values from gyro device
    
    // integrate angular velocity to get angle
    for(int ii=0; ii<3; ii++) {
      gyro_angle[ii] += (int32_t)(gyro_velocity[ii] / GYRO_THRESHOLD_DETECTION);
    }
    
   // request access to update servo2 posistion
    if(pdTRUE == xSemaphoreTake(servo2_mutex, 0)){
		// keep servo within boundary of 500 to 3000 pwm count
		if((gyro_angle[2]>=-75)&&(gyro_angle[2]<=175)){
		// convert gyro yaw angle to a pwm acceptable input
        servo2_position = (int)(1250 + (gyro_angle[2]*10)); // update servo 1 based on yaw of gyro
        }
	// release access back to system
	xSemaphoreGive(servo2_mutex);
    }
    osDelay(30);
  }
}
