  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

void servo2_task(void* argument);  

/*
 * initializes everything for task
 */
void servo2_task_init() {
  if (pdPASS != xTaskCreate (servo1_task,	"servo1", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

/*
 * pwm_task continuously changes duty_cycle of TIM5 channels 1&2 <CR>
 */
void servo2_task(void* argument) {
  static int count;
  while(1) {
	if(pdTRUE == xSemaphoreTake(servo1_mutex, 0)){
		servo2_posistion = ;
	xSemaphoreGive(servo1_mutex);
	}
    osDelay(10);
  }
}
