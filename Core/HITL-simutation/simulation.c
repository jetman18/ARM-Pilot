#include "simulation.h"
#include "stm32f1xx_hal.h"
#include "timer.h"
#include "usart.h"
#include "blackbox.h"

#define DATA_LENGTH 52
#define HITL_TIMEOUT 500 // us
const int magic_number = 305419896;

int k = 1e+6;
typedef struct{
   //float time;
   // imu data
   float roll_rate;
   float pitch_rate;
   float yaw_rate;
   float roll;
   float pitch;
   float yaw;
   
   // gps data
   float latitude;
   float longitude;
   float altitude;
   //float v_north;
   //float v_east;
   //float v_down;
}data_t;

typedef union {
  data_t get;
  char buf[DATA_LENGTH];
}pack;
static int parseMSG(const uint8_t c);

pack msg;


void HITL_thread(){
	/*
   uint8_t c;
   time_ms = millis();
	
   while (1){    
        int reOk = HAL_UART_Receive(&huart1,&c,1,1000);
        int re = parseMSG(c);
	     if(reOk != 0 || !re)
			 break;
    }
	
   dt_time_hitl = millis() - time_ms;
	*/
}


void hitl_callback(){
   static uint8_t c;
   parseMSG(c);
   HAL_UART_Receive_IT(&huart1,&c,1);
}


uint32_t dt_time_hitl;
uint32_t time_ms;
static int parseMSG(const uint8_t c){
    static int step = 0;
    static int frame_index = 3;
    static int frame_start = 0;
    static char buff[4];
    static int count = 3;
    if(frame_start == 0){
        buff[count] = c;
        if(count > 0){
            count --;
        }
        else{
            int magic = *(int*)buff;
            // shift bytes
            for(int i = 3; i > 0; i--){
                buff[i] = buff[i - 1];
               }
            if(magic == magic_number){
                frame_start = 1;
              }      
            }
    }else{
		msg.buf[frame_index] = c;
		if(frame_index == step){
			step += 4;
			frame_index = 4;
		    frame_index += step;
			if(step == 36){
			    step = 0;
				frame_index = 3;
				frame_start = 0;
                count = 3;
				dt_time_hitl = millis() - time_ms; 
				time_ms = millis();
                return 0;
			}			
		}
		frame_index --;
    }
    return -1;
}