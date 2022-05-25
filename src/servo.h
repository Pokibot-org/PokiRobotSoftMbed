#ifndef SERVO_H
#define SERVO_H

#define SERVO_PWM_PRESCAL 59 // -1 pour avoir pile 50Hz sur l'oscillo, no lo sé why
#define SERVO_PWM_PERIOD 60000 // 60 000 = 100% de pwm
#define SERVO_INIT 4250 // Anciennement 0.0645f avec PwmOUT
#define SERVO_LEFT 1900 // Anciennement 0.025f avec PwmOUT
#define SERVO_RIGHT 6550 // Anciennement 0.0645f avec PwmOUT

#define MG996R_MIN	600
#define MG996R_MAX	3500

#define SERVO0 0
#define SERVO1 1
#define SERVO2 2

#include <mbed.h>

void servosTimerInit();
void servoSetPwmDuty(int servo, int dutyCycle);

#endif