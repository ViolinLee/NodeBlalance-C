/*
 * Control.h
 *
 *  Created on: 25.09.2017
 *      Author: anonymous
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control
#define ITERM_MAX 10000

extern float PID_errorSum;
extern float PID_errorOld;
extern float PID_errorOld2;
extern float setPointOld;

float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);




#endif /* CONTROL_H_ */
