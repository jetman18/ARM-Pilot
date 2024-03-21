#ifndef SIMULATION_H
#define SIMULATION_H
#ifdef __cplusplus
extern "C" {
#endif
#include "usart.h"
void HITL_init(UART_HandleTypeDef *huart);
void HITL_thread();
void hitl_callback();
#ifdef __cplusplus
}
#endif
#endif
