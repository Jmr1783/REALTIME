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
  static int max_count = 30,min_count = 1;
  while(1) {
	if(pdTRUE == xSemaphoreTake(servo1_mutex, 0)){
		if(generate_flag){
			servo1_posistion =((RNG->DR) % (max_count+1 - min_count) + min_count); // generate random posistion
			
			generate_flag = 0;
		}
	xSemaphoreGive(servo1_mutex);
	}
	osDelay(10);
  }
}
