  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

void pwm_task(void* argument);  
void set_pwm_pulse(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse);

extern int servo1_position, servo2_position;
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;



void set_pwm_pulse(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse) {
  TIM_OC_InitTypeDef sConfigOC = {0};
    
  sConfigOC.OCMode = (channel == TIM_CHANNEL_1) ? TIM_OCMODE_PWM1 : TIM_OCMODE_PWM2;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_PWM_Start(htim, channel);
}



/*
 * initializes everything for pwm task
 */
void pwm_task_init() {
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  if (pdPASS != xTaskCreate (pwm_task,	"pwm", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

/*
 * pwm_task continuously changes duty_cycle of TIM5 channels 1&2 <CR>
 */
void pwm_task(void* argument) {
  static int servo1_count = 0, servo2_count = 0;
  while(1) {
    if(pdTRUE == xSemaphoreTake(servo1_mutex, 0)){
	servo1_count = servo1_position;
	xSemaphoreGive(servo1_mutex);
	}
	if(pdTRUE == xSemaphoreTake(servo2_mutex, 0)){
	servo2_count = servo2_position;
 	xSemaphoreGive(servo2_mutex);
	}
	
    set_pwm_pulse(&htim5, TIM_CHANNEL_1, servo1_count);
    set_pwm_pulse(&htim5, TIM_CHANNEL_2, servo2_count);
    osDelay(10);
  }
}
