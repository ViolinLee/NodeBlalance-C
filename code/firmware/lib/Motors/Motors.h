/*
 * Motors.h
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include <Arduino.h>

#define PIN_ENABLE_MOTORS 12            // port definitions
#define PIN_MOTOR_LEFT_DIR 27
#define PIN_MOTOR_LEFT_STEP 14
#define PIN_MOTOR_RIGHT_DIR 25
#define PIN_MOTOR_RIGHT_STEP 26
#define PIN_MOTOR_GEAR_DIR 32
#define PIN_MOTOR_GEAR_STEP 33

#define ZERO_SPEED 0xffffff
#define MAX_ACCEL 14               // Maximun motor acceleration (MAX RECOMMENDED VALUE: 20) (default:14)

extern volatile int32_t steps1;
extern volatile int32_t steps2;
extern volatile int32_t steps3;
extern int16_t speed_M1, speed_M2, speed_M3;        // Actual speed of motors
extern int8_t  dir_M1, dir_M2, dir_M3;            // Actual direction of steppers motors
extern hw_timer_t * timer1;
extern hw_timer_t * timer2;
extern hw_timer_t * timer3;

void setMotorSpeedM1(int16_t tspeed);
void setMotorSpeedM2(int16_t tspeed);
void setMotorSpeedM3(int16_t tspeed);

void initTimers();

#endif
