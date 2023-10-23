#ifndef _CONTROLLER_
#define _CONTROLLER_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void pitchRollControl(float target_roll,float target_pitch);
void setPIDgain(int axis,float kp,float ki,float kd);


#ifdef __cplusplus
}
#endif
#endif