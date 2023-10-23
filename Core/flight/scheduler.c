#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "scheduler.h"
////////////////////////
#include "../lib/log.h"
#include "../lib/imu.h"
#include "../lib/pwmwrite.h"
#include "../lib/pid.h"
#include "../Driver/ibus.h"
#include "../lib/gps.h"
#include "../lib/imu.h"
#include "../lib/timer.h"
///////////////////////////////
#include "../Driver/mpu6500.h"
#include "../Driver/ms5611.h"
#include "../Driver/qmc5883.h"
///////////////////////////////
#include "../flight/mavlink_handler.h"


#define LOOP_US  4000
#define MAX_LOOP_BREAK_US  3900
uint32_t max_excution_time_us;
uint32_t num_tasks;

task_t task[]={ 
    {ahrs_update,      0,0,0,1}, /*  imu task 500 hz*/

  //{pidUpdate,       0,0,0,1},/*  pid task  500 hz*/

  //{ms5611_start,         0,0,0,2},/*  pid task  5 hz*/

  //{pwm2esc,         0,0,0,1},/*  esc task  500 hz*/

  //{ibusGet,         0,0,0,10},/*  receiver task  50 hz*/
};

void init_sche(){
	ms5611_init(&hi2c1);
	timer_start(&htim4);
	initPWM(&htim3);
	ibusInit(&huart2,115200);
	//mavlinkInit(SYS_ID,0,&huart2,115200);
	mpu_init();
	motoIdle();
	qmc5883_init(&hi2c1);
    //bmp280_init();
	num_tasks = 0;
	num_tasks  = sizeof(task)/sizeof(task_t);
	max_excution_time_us = 0;
}

static void wait(){
    static uint32_t time_us;
    while(( micros() - time_us )<LOOP_US);
    time_us = micros();
}
void start_scheduler() {
  static int counter = 0;
  uint32_t time_1;
  uint32_t total_execution_time_us = 0;
  for (int i = 0; i < num_tasks; i++){
      if((task[i].exec != NULL) && (counter % task[i].period == 0)){
        time_1 = micros();
        task[i].execution_cycle_us = micros() - task[i].last_exec_time_us;
        task[i].last_exec_time_us = time_1;
        task[i].exec();
        task[i].execution_time_us = micros() - time_1;
        total_execution_time_us += task[i].execution_time_us;
        if(total_execution_time_us > MAX_LOOP_BREAK_US){
          break;
        }
      }
  }
  max_excution_time_us = total_execution_time_us;
  counter ++;
  if(counter == 3999)counter = 0;
  wait(); 
}

