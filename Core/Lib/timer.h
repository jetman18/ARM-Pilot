
#ifndef LIB_TIMER_H_
#define LIB_TIMER_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "stdio.h"
typedef struct{
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
}bootTime_t;

extern TIM_HandleTypeDef *htimmz;
static volatile uint32_t micross;
#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000UL)
#define TIMER_CALLBACK() (micross += 65535UL)
uint32_t micros();
uint32_t millis();
void timer_start(TIM_HandleTypeDef *htimz);
void delay_ms(uint32_t val);
void delay_us(uint32_t val);
#ifdef __cplusplus
}
#endif
#endif
