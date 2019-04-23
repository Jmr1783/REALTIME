/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"
#include "../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"

#define GYRO_THRESHOLD_DETECTION (10000)   	/* Tunnng parameters for gyroscope */

extern int servo1_position,servo2_position;
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;
/* Gyroscope variables */
float gyro_velocity[3] = {0};       // angular velocity
int32_t gyro_angle[3] = {0, 0, 0};	// angle

void gyro_task(void* argument);  

void gyro_task_init() {
  
  if(BSP_GYRO_Init() != HAL_OK)  {
    /* Initialization Error */
    Error_Handler();
  }
  if (pdPASS != xTaskCreate (gyro_task,	"print", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

void gyro_task(void* argument) {
  static char buf[100];
  static int servo1_print=0,servo2_print=0;
  while(1) {
   
   if(pdTRUE == xSemaphoreTake(servo1_mutex, 0)){
      servo1_print = (servo1_position+100)%20000;
      xSemaphoreGive(servo1_mutex);
   }
     
		BSP_GYRO_GetXYZ(gyro_velocity);   // get raw values from gyro device
    
    // integrate angular velocity to get angle
    for(int ii=0; ii<3; ii++) {
      gyro_angle[ii] += (int32_t)(gyro_velocity[ii] / GYRO_THRESHOLD_DETECTION);
    }
    
   // update servo2 posistion
	if(pdTRUE == xSemaphoreTake(servo2_mutex, 0)){
	servo2_position = (gyro_angle[2]+100)%20000; // update servo 1 based on yaw of gyro
   servo2_print = servo2_position;
	xSemaphoreGive(servo2_mutex);
	}
    
	//count = (count+100) % 20000;
	//servo2_position = (gyro_angle[2]+100)%20000; // needs math to convert to count
	 
    sprintf(buf, "%d\t%d\t%d\t%d\t%d\t\n\r", gyro_angle[0], gyro_angle[1], gyro_angle[2],servo1_print,servo2_print);
    vPrintString(buf);
    osDelay(30);
  }
}
