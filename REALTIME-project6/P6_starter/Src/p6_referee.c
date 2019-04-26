  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
//****************************************************************************************************
//Function prototype section
void referee_task(void* argument);  
//****************************************************************************************************
//Global variables section
extern int servo1_position, servo2_position;
extern int32_t gyro_angle[3];
//****************************************************************************************************
//Type Definitions section
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;
/*
 * initializes everything for task
 */
void referee_task_init() {
  if (pdPASS != xTaskCreate (referee_task,  "referee", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

//****************************************************************************************************
/*
	FUNCTION THAT HANDLES THE GAMES OUTCOME AND MANAGES THE MOVEMENT OF SERVO1
	INPUT PARAMETERS: pvParameters THAT IS SENT BY THE RTOS PROGRAM
	FUNCTION OUTPUT: void OUTPUT TYPE
*/
void referee_task(void* argument) {
  static char buf[100];
  static int max_count = 3000,min_count = 500, yaw_pos_tolerance =0, yaw_neg_tolerance, hits=0,misses=0, 
             tick_cnt=0,finished_generation_t=0,timer_cnt=0,round_cnt=1;
  static uint8_t generate_flag = 0,hit_flag=0,game_running=1;
  
	while(1) {	
		if(round_cnt == 1){ // initial generate 
		   servo1_position =((RNG->DR) % (max_count+1 - min_count) + min_count); // generate random posistion  
           finished_generation_t = xTaskGetTickCount();
		}
		while(game_running){
			//After round ten, print out stats
			if(round_cnt==11){
				sprintf(buf, "\f Game Over...\n\r Your stats are as follows:\n\rHits: %d\n\rMisses: %d\n\r", hits, misses);
				vPrintString(buf);
				game_running=0;
			}
			//Get current system tick count 1ms = 1 tick
			tick_cnt = xTaskGetTickCount(); 	
			//If software timer hits 5 secs 
			if(generate_flag){
					servo1_position =((RNG->DR) % (max_count+1 - min_count) + min_count); // generate new random servo1 posistion  
					finished_generation_t = xTaskGetTickCount(); // record when this action was completed
					generate_flag = 0;	// reset flag for next generation
					if(hit_flag)
						hit_flag=0; // reset flag for next round
			}
			//If software timer has not expired
			else{
					// update timer count
					timer_cnt = tick_cnt-finished_generation_t;		
					// check if timer has hit 5 secs
					if(timer_cnt >= 5000){
							generate_flag = 1; // generate new position
							round_cnt++;
						//Did we servo 1 and servo 2 not meet tolerance?
						if(!hit_flag){
							misses++;
							if(round_cnt==11){
								sprintf(buf, "\f You MISSED the target......\n\r");
								vPrintString(buf);
							}
							else{
							sprintf(buf, "\f You MISSED the target......\n\r Round: %d",round_cnt);
							vPrintString(buf);
							}
							osDelay(2000);  // wait 2 seconds to give the user time to cool off
						}
						
					}
					
					//check what the player's posistion is
					else{
						// calculate the current acceptable range of posistion necessary to hit a target
						yaw_pos_tolerance = servo1_position + (servo1_position*0.05);
						yaw_neg_tolerance = servo1_position - (servo1_position*0.05);
						// Tell the user where they are as a display
						sprintf(buf, "\f  Servo1: %d Servo2: %d Round: %d Hits: %d Misses: %d\n\r", 
						servo1_position, servo2_position,round_cnt,hits,misses);
						vPrintString(buf);		 
						// Use tolerance to decide if we hit our target and if so tell the user
						if((yaw_pos_tolerance>=servo2_position)&&(yaw_neg_tolerance<=servo2_position)){
							sprintf(buf, "\f You Hit the target......\n\r Round: %d",round_cnt);
							vPrintString(buf);
							hits++;
							round_cnt++;
							hit_flag=1;
							osDelay(2000); // wait 2 seconds to give the user time to cool off
							generate_flag = 1;
						}
					}
			} 
			osDelay(10);
	  }
		osDelay(30);
  }
}
