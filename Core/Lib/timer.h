
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
#define HZ_TO_MICRO(hz)  (uint32_t)(((1.0f)/(hz))*1000000UL)
//#define micros() (uint32_t)((micross) + (__HAL_TIM_GET_COUNTER(htimmz)))
//#define millis()  (uint32_t)(micross/1000UL)
uint32_t micros();
uint32_t millis();
void timer_callback();
void timer_start(TIM_HandleTypeDef *htimz);
void delay_ms(uint32_t val);
void delay_us(uint32_t val);
#ifdef __cplusplus
}
#endif
#endif
