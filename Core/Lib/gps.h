#ifndef _GPS_H_
#define _GPS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f1xx_hal.h"

void gpsInit(UART_HandleTypeDef *uartt,uint32_t baudrate);
void gpsCallback(void);
uint8_t getSat();
uint8_t  getFix();
uint32_t getUpdateTime();
#ifdef __cplusplus
}
#endif
#endif
