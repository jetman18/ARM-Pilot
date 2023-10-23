#ifndef _INTERRUPTHANDLER_H_
#define _INTERRUPTHANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal.h"
/////////////////////////
#include "../flight/mavlink_handler.h"
/////////////////////////
#include "../Driver/ibus.h"
#include "../lib/timer.h"
#include "../lib/ppmreceiver.h"
// IQR function
//----------------------------------IQR--Handler-----------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/* ibus */
    if(huart == &huart1)
	{
        ibusCallback();
    }
    else if(huart == &huart1)
    {
      	//mavlinkCallback();
    	//flowCallback();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	// ibus
    if(huart == &huart1)
	{
    	mav_tx_cpl_callback();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_8){
        //hc_sr04_callback();
    }
 /*
   else if(GPIO_Pin == GPIO_PIN_0){
        ppmcallback();
    }
    */
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim4)
	{
		TIMER_CALLBACK();
	}
}
//----------------------------------IQR--Handler-----------------------------



#ifdef __cplusplus
}
#endif

#endif
