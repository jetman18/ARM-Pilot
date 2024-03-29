#ifndef MAINLOOP_H
#define MAINLOOP_H
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

// navigation
extern uint16_t autopilot_stick;
extern uint16_t circle_stick;
extern uint16_t rtHome_stick;
void Autopilot();

// mavlink handler
void mavlinkInit(uint8_t syss_id, uint8_t comm_id,UART_HandleTypeDef *uartt,uint32_t baudrate);
void mavlinkCallback();
void mav_tx_cpl_callback();
void mavlink_send();

// controller
void pitchRollControl(float target_roll,float target_pitch);
void setPIDgain(int axis,float kp,float ki,float kd);

// mainloop
void main_loop();
#ifdef __cplusplus
}
#endif

#endif

