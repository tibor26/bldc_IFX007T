#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"


void commutation(uint8_t direction, uint8_t duty_cycle, uint8_t field_weakening);
void init_pwm(void);
void PID_control(uint8_t direction, uint16_t speed);
void PID_init(int32_t kp, int32_t ki, int32_t kd, uint8_t direction, uint8_t output_init);


#ifdef __cplusplus
}
#endif

#endif // !__MOTOR_CONTROL_H
