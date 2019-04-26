  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

void servo1_task(void* argument);  

/*
 * initializes everything for task
 */
void servo1_task_init() {
  if (pdPASS != xTaskCreate (servo1_task,	"servo1", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

/*
 * pwm_task continuously changes duty_cycle of TIM5 channels 1&2 <CR>
 */
void servo1_task(void* argument) {
  static int count;
  while(1) {
	// your code here
    osDelay(10);
  }
}
