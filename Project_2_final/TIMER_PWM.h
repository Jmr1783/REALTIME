#ifndef __TIMER_PWM_H
#define __TIMER_PWM_H

#include "stm321476xx.h"
#include "SysClock.h"

#define SERVO1 0
#define SERVO2 1

void timer2_pwm_init(uint32_t base);
void move_servo(uint8_t servo_select, uint32_t pulse_width);

#endif