#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SERVO1 0
#define SERVO2 1
#define MOV 0x20
#define WAIT 0x40
#define LOOP 0x80
#define END_LOOP 0xA0
#define RECIPE_END 0x00
#define POSITION_0 20
#define POSITION_1 17
#define POSITION_2 14
#define POSITION_3 11
#define POSITION_4 8
#define POSITION_5 4


enum events
{
	user_paused,
	user_continued,
	user_entered_right,
	user_entered_left,
	user_no_op,
	user_start,
	recipe_ended
};

enum status
{
	status_running,
	status_paused,
	status_command_error,
	status_nested_error
};

enum servo_states
{
	state_at_position,
	state_unknown,
	state_moving,
	state_recipe_ended
};

static enum servo_states current_servo_state[2] = {state_unknown, state_unknown};
static enum status current_servo_status[2] = {status_running, status_running};
static enum events servo1_events, servo2_events;

//Several recipes to be used in demo
unsigned char recipe0[] = {MOV | 0, MOV | 5, MOV | 0, MOV | 3, LOOP | 0, MOV | 1, MOV | 4, END_LOOP, MOV | 0, MOV | 2, WAIT | 0, MOV | 3, WAIT | 0, MOV | 2, MOV | 3, WAIT | 31, WAIT | 31, WAIT | 31, MOV | 4, RECIPE_END};
unsigned char recipe1[] = {MOV | 0, MOV | 1, MOV | 2, MOV | 3, MOV | 4, MOV | 5, MOV |4, MOV | 3, MOV | 2, MOV |1, RECIPE_END};
unsigned char recipe2[] = {MOV | 3, MOV | 1, MOV | 3, RECIPE_END, MOV | 5};
unsigned char recipe3[] = {MOV | 2, WAIT | 50, MOV | 2, MOV | 7, WAIT | 10, 0x60, RECIPE_END};
unsigned char servo_recipe_index[2] = {0, 0};
unsigned char servo_position[2] = {0, 0};

void timer2_pwm_init(void);
void timer3_init(void);
void move_servo(uint8_t servo_select, uint32_t position);
void main_control(uint8_t servo_select);
void check_status_leds(uint8_t servo_select);
void process_user_commands(void);
void event_handler_servo1(uint8_t servo_select);
void event_handler_servo2(uint8_t servo_select);

//Variable needed to track "wait" duration
unsigned char waitCount[2] = {0, 0};

//Variables needed to track "loop" conditions
unsigned char loop_count[2] = {0, 0};
unsigned char loop_start[2] = {0, 0};
unsigned char nested_check[2] = {0, 0};

//test
unsigned char test[3] = "MOV";

int main (void)
{
	//Initialize the microcontroller;
	System_Clock_Init();
	UART2_Init();
	LED_Init();
	timer2_pwm_init();
	timer3_init();
	
	while(1)
	{
		TIM3->CNT = 0;
		
		//Allow user to enter commands
		process_user_commands();
		
		//Apply recipe commands to servo 1
		main_control(SERVO1);
		check_status_leds(SERVO1);
		
		//Apply recipe commands to servo 2
		main_control(SERVO2);
		
		while(1)
		{
			if (TIM3->CNT >= 1000)
				break;
		}
	}
}

//Completely initialize TIM2 CH1 and CH2 for PWM output
void timer2_pwm_init(void)
{
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
	TIM2->PSC = 8000;
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
	TIM2->ARR = 200;
	TIM2->CCR1 = 50; //Default CH1 to 25% duty cycle
	TIM2->CCR2 = 50; //Default CH2 to 25% duty cycle
	//Set update event for prescaler
	TIM2->EGR |= TIM_EGR_UG;
	//Clear extraneous update flag
	TIM2->SR &= ~TIM_SR_UIF;
	//Enable TIM2 to run indefinitely.
	TIM2->CR1 = 0x1;
	
	return;
}

void timer3_init()
{
    /*
      Use update event to check for overflow
      Set in Upcounting mode
      Counts from 0 to 1000
      100us period * 1000 counts = 100ms
    */

    //Enable clock to TIM3
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;
    //Set TIM3 prescaler to 8000. 1/ fCK_PSC / (PSC[15:0] + 1) = 1/(80MHz / (7999 + 1) ) = 100us period
    TIM3->PSC = 7999;
    //Set the auto reload preload functionality
    TIM3->CR1 |= 1<<7;
    //Set auto reload register to 1000 for 100ms
    TIM3->ARR = 5000;
    //Set update event for prescaler
    TIM3->EGR |= TIM_EGR_UG;
    //Clear extraneous update flag
    TIM3->SR &= ~TIM_SR_UIF;
    //Enable TIM3 to run indefinitely.
    TIM3->CR1 = 0x1;

    return;
}

void move_servo(uint8_t servo_select, uint32_t position)
{
	uint32_t pwm_position;
	
	//Assign pwm_positon based on command arguement which will hold proper CCR regsiter value for pwm
	switch(position)
	{
		case(0):
		{
			pwm_position = POSITION_0;
			break;
		}
		case(1):
		{
			pwm_position = POSITION_1;
			break;
		}
		case(2):
		{
			pwm_position = POSITION_2;
			break;
		}
		case(3):
		{
			pwm_position = POSITION_3;
			break;
		}
		case(4):
		{
			pwm_position = POSITION_4;
			break;
		}
		case(5):
		{
			pwm_position = POSITION_5;
			break;
		}
	}
	
	//Assign the value for PWM to the CCR register depending on which servo we are operation on
	if (position > 31)
		current_servo_status[servo_select] = status_command_error;
	else if (servo_select == SERVO1)
	{
		TIM2->CCR1 = pwm_position;
		servo_position[servo_select] = position;
	}
	else if (servo_select == SERVO2)
	{
		TIM2->CCR2 = pwm_position;
		servo_position[servo_select] = position;
	}
	
	return;
}

void main_control(uint8_t servo_select)
{
	//Check if current recipe is waiting
	if (waitCount[servo_select])
	{
		waitCount[servo_select]--;
		return;
	}
	
	//Check if the recipe has been paused or ended already
	if ((current_servo_state[servo_select] == state_recipe_ended) || (current_servo_status[servo_select] == status_paused))
		return;
	
	//Decode the nest command in the recipe
	unsigned char op_code, range;
	op_code = recipe0[servo_recipe_index[servo_select]] & 0xE0;
	range = recipe0[servo_recipe_index[servo_select]] & 0x1F;
	
	//Execute the op code
	switch(op_code)
	{
		case(MOV):
		{
			move_servo(servo_select, range);
			current_servo_status[servo_select] = status_running;
			break;
		}
		case(WAIT):
		{
			waitCount[servo_select] = range;
			current_servo_status[servo_select] = status_running;
			break;
		}
		case(LOOP):
		{
			if (nested_check[servo_select])
			{
				current_servo_status[servo_select] = status_nested_error;
				break;
			}
			current_servo_status[servo_select] = status_running;
			loop_count[servo_select] = range;
			loop_start[servo_select] = servo_recipe_index[servo_select];
			nested_check[servo_select] = 1;
			break;
		}
		case(END_LOOP):
		{
			if (loop_count[servo_select])
			{
				current_servo_status[servo_select] = status_running;
				loop_count[servo_select]--;
				servo_recipe_index[servo_select] = loop_start[servo_select];
			}
			else
			{
				current_servo_status[servo_select] = status_running;
				loop_start[servo_select] = NULL;
				nested_check[servo_select] = 0;
			}
			break;
		}
		case(RECIPE_END):
		{
			current_servo_status[servo_select] = status_running;
			current_servo_state[servo_select] = state_recipe_ended;
			break;
		}
		default:
		{
			current_servo_status[servo_select] = status_command_error;
			break;
		}
	}
	servo_recipe_index[servo_select]++;
	return;
}

void check_status_leds(uint8_t servo_select)
{
	
	switch(current_servo_status[servo_select])
	{
		case(status_running):
		{
			Green_LED_On();
			Red_LED_Off();
			break;
		}
		case(status_paused):
		{
			Green_LED_Off();
			Red_LED_Off();
			break;
		}
		case(status_command_error):
		{
			Green_LED_Off();
			Red_LED_On();
			break;
		}
		case(status_nested_error):
		{
			Green_LED_On();
			Red_LED_On();
			break;
		}
		default:
		{
			Green_LED_On();
			Red_LED_On();
			break;
		}
	}
	
	return;
}

char user_input[4];
uint8_t rxByte;
char i = 0;

void process_user_commands(void)
{
	char msg[5];
	
	if (i == 0)
	{
		sprintf(msg, "\n\r> ");
		USART_Write(USART2, (uint8_t *)msg, strlen(msg));
		i++;
	}
	
	if (USART2->ISR & USART_ISR_RXNE)
	{
		rxByte = USART_Read(USART2);
	    if (rxByte != 0x0D)
		{	
			USART_Write(USART2, &rxByte, 1);
		}
		user_input[i] = rxByte;
		i++;
	}
	
	if (i == 4)
	{
	switch(user_input[1])
	{
		case 'p':
		case 'P':
		{
			if (current_servo_state[SERVO1] == state_recipe_ended)
			{
				servo1_events = recipe_ended;
				current_servo_status[SERVO1] = status_paused;
			}
			else if (current_servo_status[SERVO1] == status_command_error)
				servo1_events = user_no_op;
			else
			{
				servo1_events = user_paused;
				current_servo_status[SERVO1] = status_paused;
			}
			break;
		}
		case 'c':
		case 'C':
		{
			if (current_servo_state[SERVO1] == state_recipe_ended)
				servo1_events = recipe_ended;
			else if (current_servo_status[SERVO1] == status_command_error)
				servo1_events = user_no_op;
			else
				servo1_events = user_continued;
			break;
		}
		case 'r':
		case 'R':
		{
			if (current_servo_status[SERVO1] != status_paused)
				servo1_events = user_no_op;
			else if (servo_position[SERVO1] > 0)
				servo1_events = user_entered_right;
			else
				servo1_events = user_no_op;
			break;
		}
		case 'l':
		case 'L':
		{
			if (current_servo_status[SERVO1] != status_paused)
				servo1_events = user_no_op;
			else if (servo_position[SERVO1] < 5)
				servo1_events = user_entered_left;
			else
				servo1_events = user_no_op;
			break;
		}
		case 'n':
		case 'N':
		{
			servo1_events = user_no_op;
			break;
		}
		case 'b':
		case 'B':
		{
			servo1_events = user_start;
			break;
		}
		case 'x':
		case 'X':
		{
			process_user_commands();
			break;
		}
		default:
		{
			return;
		}
	}
	
		switch(user_input[2])
	{
		case 'p':
		case 'P':
		{
			if (current_servo_state[SERVO2] == state_recipe_ended)
			{
				servo2_events = recipe_ended;
				current_servo_status[SERVO2] = status_paused;
			}
			else if (current_servo_status[SERVO2] == status_command_error)
				servo1_events = user_no_op;
			else
			{
				servo2_events = user_paused;
				current_servo_status[SERVO2] = status_paused;
			}
			break;
		}
		case 'c':
		case 'C':
		{
			if (current_servo_state[SERVO2] == state_recipe_ended)
				servo2_events = recipe_ended;
			else if (current_servo_status[SERVO2] == status_command_error)
				servo1_events = user_no_op;
			else
				servo2_events = user_continued;
			break;
		}
		case 'r':
		case 'R':
		{
			if (current_servo_status[SERVO2] != status_paused)
				servo2_events = user_no_op;
			else if (servo_position[SERVO2] > 0)
				servo2_events = user_entered_right;
			else
				servo2_events = user_no_op;
			break;
		}
		case 'l':
		case 'L':
		{
			if (current_servo_status[SERVO2] != status_paused)
				servo2_events = user_no_op;
			else if (servo_position[SERVO2] < 5)
				servo2_events = user_entered_left;
			else
				servo2_events = user_no_op;
			break;
		}
		case 'n':
		case 'N':
		{
			servo2_events = user_no_op;
			break;
		}
		case 'b':
		case 'B':
		{
			servo2_events = user_start;
			break;
		}
		case 'x':
		case 'X':
		{
			process_user_commands();
			break;
		}
		default:
		{
			return;
		}
	}
	i = 0;
	event_handler_servo1(SERVO1);
	event_handler_servo2(SERVO2);
	}
	return;
}

void event_handler_servo1(uint8_t servo_select)
{
	switch(current_servo_state[servo_select])
	{
		case (state_at_position):
		{
			if (servo1_events == user_paused) // if events returned from user_command() is user_paused
			{			
				current_servo_state[servo_select] = state_at_position; // the state of the servo stays the same
				current_servo_status[servo_select] = status_paused;    // the status of the servo is paused	
			}
			else if(servo1_events == user_continued) // if events returned from user_command() is user_continued
			{
				current_servo_state[servo_select] = state_moving;	// the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == user_entered_right) // if events returned from user_command() is user_entered_right
			{
				servo_position[servo_select]--; // decrease the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the right
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving	
				current_servo_status[servo_select] = status_running; // the state of the servo is running		
			}
			else if (servo1_events == user_entered_left) // if events returned from user_command() is user_entered_left
			{
				servo_position[servo_select]++; // increase the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the left
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving		
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}	
			else if(servo1_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_at_position; // stay at the same state
				current_servo_status[servo_select] = status_paused;  // the status of the servo is paused	at its position
			}
			else if (servo1_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //The state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == recipe_ended) // if events returned from user_command() is reciped_ended
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo is recipe ended
				current_servo_status[servo_select] = status_paused; // the satus of the servo is paused
			}
			break;
		}
		case state_unknown:
			break;
		case (state_moving):
		{
			if (servo1_events == user_paused) // if events returned from user_command() is user_paused
			{			
				current_servo_state[servo_select] = state_at_position; // the state of the servo stops moving stays at the position it was in
				current_servo_status[servo_select] = status_paused;  // the status of the servo is paused	
			}
			else if(servo1_events == user_continued) // if events returned from user_command() is user_continued
			{
				current_servo_state[servo_select] = state_moving;	// the state of the servo is moving; stays the same
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == user_entered_right) // if events returned from user_command() is user_entered_right
			{
				servo_position[servo_select]--; // decrease the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the right
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving	
				current_servo_status[servo_select] = status_running; // the state of the servo is running		
			}
			else if (servo1_events == user_entered_left) // if events returned from user_command() is user_entered_left
			{
				servo_position[servo_select]++; // increase the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the left
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving		
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}	
			else if(servo1_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_moving; // stay at the same state
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == recipe_ended) // if events returned from user_command() is reciped_ended
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo is recipe ended
				current_servo_status[servo_select] = status_paused; // the status of the servo is paused
			}
			break;
		}
		case (state_recipe_ended):
		{
			if(servo1_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_recipe_ended; // stay at the same state
				current_servo_status[servo_select] = status_paused; // the status of the servo is paused
			}
			else if (servo1_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else // recipe ended cannot be exited on any other states/statuses
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo stays the same
				current_servo_status[servo_select] = status_paused; // the status of the servo stays the same
			}
			break;
		}
	}
	return;
}


void event_handler_servo2(uint8_t servo_select)
{
	switch(current_servo_state[servo_select])
	{
		case (state_at_position):
		{
						if (servo1_events == user_paused) // if events returned from user_command() is user_paused
			{			
				current_servo_state[servo_select] = state_at_position; // the state of the servo stays the same
				current_servo_status[servo_select] = status_paused;    // the status of the servo is paused	
			}
			else if(servo1_events == user_continued) // if events returned from user_command() is user_continued
			{
				current_servo_state[servo_select] = state_moving;	// the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == user_entered_right) // if events returned from user_command() is user_entered_right
			{
				servo_position[servo_select]--; // decrease the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the right
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving	
				current_servo_status[servo_select] = status_running; // the state of the servo is running		
			}
			else if (servo1_events == user_entered_left) // if events returned from user_command() is user_entered_left
			{
				servo_position[servo_select]++; // increase the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the left
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving		
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}	
			else if(servo1_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_at_position; // stay at the same state
				current_servo_status[servo_select] = status_paused;  // the status of the servo is paused	at its position
			}
			else if (servo1_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //The state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo1_events == recipe_ended) // if events returned from user_command() is reciped_ended
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo is recipe ended
				current_servo_status[servo_select] = status_paused; // the satus of the servo is paused
			}
			break;
		}
		case state_unknown:
			break;
		case (state_moving):
		{
			if (servo2_events == user_paused) // if events returned from user_command() is user_paused
			{			
				current_servo_state[servo_select] = state_at_position; // the state of the servo stops moving stays at the position it was in
				current_servo_status[servo_select] = status_paused;  // the status of the servo is paused	
			}
			else if(servo2_events == user_continued) // if events returned from user_command() is user_continued
			{
				current_servo_state[servo_select] = state_moving;	// the state of the servo is moving; stays the same
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo2_events == user_entered_right) // if events returned from user_command() is user_entered_right
			{
				servo_position[servo_select]--; // decrease the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the right
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving	
				current_servo_status[servo_select] = status_running; // the state of the servo is running		
			}
			else if (servo2_events == user_entered_left) // if events returned from user_command() is user_entered_left
			{
				servo_position[servo_select]++; // increase the servo position
				move_servo(servo_select, servo_position[servo_select]); // execute the move to the left
				current_servo_state[servo_select] = state_moving; // the state of the servo is moving		
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}	
			else if(servo2_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_moving; // stay at the same state
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo2_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else if (servo2_events == recipe_ended) // if events returned from user_command() is reciped_ended
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo is recipe ended
				current_servo_status[servo_select] = status_paused; // the status of the servo is paused
			}
			break;
		}
		case (state_recipe_ended):
		{
			if(servo2_events == user_no_op) // if events returned from user_command() is user_no_op
			{
				current_servo_state[servo_select] = state_recipe_ended; // stay at the same state
				current_servo_status[servo_select] = status_paused; // the status of the servo is paused
			}
			else if (servo2_events == user_start) // if events returned from user_command() is user_start
			{
				current_servo_state[servo_select] = state_moving; //the state of the servo is moving
				current_servo_status[servo_select] = status_running; // the status of the servo is running
			}
			else // recipe ended cannot be exited on any other states/statuses
			{
				current_servo_state[servo_select] = state_recipe_ended; // the state of the servo stays the same
				current_servo_status[servo_select] = status_paused; // the status of the servo stays the same
			}
			break;
		}
	}
	return;
}
