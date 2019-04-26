  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "stdio.h"
#include "string.h"

void pwm_task(void* argument);  
void set_pwm_pulse(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t pulse);

extern int servo1_position, servo2_position;
extern SemaphoreHandle_t servo1_mutex, servo2_mutex;

/*
 * initializes everything for pwm task
 */
void pwm_task_init() {
  
	//Completely initialize TIM2 CH1 and CH2 for PWM output

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
	TIM2->PSC = 79;
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
	TIM2->ARR = 20000;
	TIM2->CCR1 = 50; //Default CH1 to 25% duty cycle
	TIM2->CCR2 = 50; //Default CH2 to 25% duty cycle
	//Set update event for prescaler
	TIM2->EGR |= TIM_EGR_UG;
	//Clear extraneous update flag
	TIM2->SR &= ~TIM_SR_UIF;
	//Enable TIM2 to run indefinitely.
	TIM2->CR1 = 0x1;
	
	if (pdPASS != xTaskCreate (pwm_task,	"pwm", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

/*
 * pwm_task continuously changes duty_cycle of TIM2 channels 1&2 <CR>
 */
void pwm_task(void* argument) {
  while(1) {
		TIM2->CCR1 = servo1_position;
		TIM2->CCR2 = servo2_position;
		osDelay(10);
  }
}
