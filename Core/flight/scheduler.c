#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "scheduler.h"
#include "plane.h"

#include "log.h"
#include "imu.h"
#include "pwmwrite.h"
#include "pid.h"
#include "ibus.h"
#include "gps.h"
#include "imu.h"
#include "timer.h"

#include "mpu6050.h"
#include "ms5611.h"
#include "qmc5883.h"
#include "interrupthandler.h"

#define LOOP_US  4000
static uint32_t mil;
uint32_t max_excution_time_us;
uint32_t num_tasks;
uint16_t F_loop;
void tongepin(){
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}
task_t task[]={ 
   // {ahrs_update,      0,0,0,1}, /*  imu task 500 hz*/

  //{pidUpdate,       0,0,0,1},/*  pid task  500 hz*/

  //{ms5611_start,         0,0,0,2},/*  pid task  5 hz*/

  //{pwm2esc,         0,0,0,1},/*  esc task  500 hz*/
   {tongepin,         0,0,0,250}
  //{ibusGet,         0,0,0,10},/*  receiver task  50 hz*/
};

void init_scheduler(){

	//ms5611_init(&hi2c2);
	timer_start(&htim4);
	//initPWM(&htim3);
    //ibusInit(&huart2,115200);
	//mavlinkInit(SYS_ID,0,&huart2,115200);
	//motoIdle();
	//qmc5883_init(&hi2c1);
    //bmp280_init();
	//blackboxInit();
	/*
	int8_t connect = mpu6050_init(&hi2c2);
	if(connect){
	   while(1){
		   HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		   HAL_Delay(100);
	   }
	mpu_calibrate();
	}
	
	*/
	num_tasks = 0;
	num_tasks  = sizeof(task)/sizeof(task_t);
	max_excution_time_us = 0;
	F_loop = 1/(LOOP_US*(1e-6f));
}

uint32_t time_us;
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
      }
  }
  max_excution_time_us = total_execution_time_us;
  counter ++;
  if(counter ==  F_loop)counter = 0;
  while(( micros() - time_us )<LOOP_US);
  time_us = micros();
}

