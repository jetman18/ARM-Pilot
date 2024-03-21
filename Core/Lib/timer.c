#include "timer.h"
#include "stm32f1xx_hal.h"
#include "maths.h"

TIM_HandleTypeDef *htimmz;
bootTime_t boottime;
static uint16_t setoverFlow(int val,int flow_val);
uint32_t _micros;

void delay_us(const uint32_t val)
{
  const uint32_t time_usss = micros();
  while(1){
    int dt = (int)( micros() - time_usss);
	if( abs(dt) > val)
		break;
  }
}

void delay_ms(const uint32_t val)
{
	delay_us(val*1000);
}
static uint16_t setoverFlow(int val,int flow_val){
    uint8_t k,l;
    l =flow_val + 1;
    k = val/l;
    k = val - (l*k);
    return k;
}
/*
uint32_t micros()
{
	return  _micros + __HAL_TIM_GET_COUNTER(htimmz);
}
*/
/*
uint32_t millis()
{
   return (uint32_t)( micros() / 1000);
}
*/
void time_inf(){
  static uint16_t sec_L  =0;
  sec_L = millis()/1000;
  boottime.sec   = setoverFlow(sec_L,59);
  boottime.min   = setoverFlow((sec_L/60),59);
  boottime.hour  = setoverFlow((sec_L/3600),23);
}

void timer_start(TIM_HandleTypeDef *htimz){
	htimmz = htimz;
	HAL_TIM_Base_Start_IT(htimmz);
}

void resetCounter(){
   __HAL_TIM_SET_COUNTER(htimmz,0);
}
void timer_callback(){
    _micros += 65536;
}

