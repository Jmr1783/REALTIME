  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

void referee_task(void* argument);  

extern int servo1_position, servo2_position;
extern int32_t gyro_angle[3];
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;

/*
 * initializes everything for task
 */
void referee_task_init() {
  if (pdPASS != xTaskCreate (referee_task,	"referee", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

/*
 * 
 */
void referee_task(void* argument) {
  static char buf[100];
  static int max_count = 30,min_count = 1, yaw_pos_tolerance =0, yaw_neg_tolerance, hits=0,misses=0, 
             tick_cnt=0,finished_generation=0,timer_cnt=0,round_cnt=0;
  static uint8_t generate_flag = 1;
  while(1) {
	
	//get current system tick count 1ms = 1 tick
    tick_cnt = xTaskGetTickCount(); 
	
	// if software timer hits 5 secs 
	if(generate_flag){
		if(pdTRUE == xSemaphoreTake(servo1_mutex, 0)){
			servo1_position =((RNG->DR) % (max_count+1 - min_count) + min_count); // generate random posistion
			xSemaphoreGive(servo1_mutex);
		}
		finished_generation = xTaskGetTickCount();
      sprintf(buf, "Your stats are as follows:\n\rHits: %d\n\rMisses: %d\n\r", hits, misses);
      vPrintString(buf);
      misses++;
		generate_flag = 0;
	}
	
	// if software timer has not expired
	else{
		
		// update timer count
		timer_cnt = tick_cnt-finished_generation;
		
		// check if timer has hit 5 secs
		if(timer_cnt >= 5000){
			generate_flag = 1;
		}
		
		//check what the player's posistion is
		else{
			yaw_pos_tolerance = servo1_position + (servo1_position*0.05);
			yaw_neg_tolerance = servo1_position - (servo1_position*0.05);
			if((yaw_pos_tolerance>=servo2_position)&&(yaw_neg_tolerance<=servo2_position)){
            sprintf(buf, "Your stats are as follows:\n\rHits: %d\n\rMisses: %d\n\r", hits, misses);
            vPrintString(buf);
            hits++;
         }
		}
	}
	
	if(round_cnt==10){
    sprintf(buf, "Your stats are as follows:\n\rHits: %d\n\rMisses: %d\n\r", hits, misses);
    vPrintString(buf);
	}
    osDelay(30);
  }
}
