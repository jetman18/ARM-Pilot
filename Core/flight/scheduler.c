#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "scheduler.h"
#include "plane.h"
#include "simulation.h"
#include "redefine.h"

#include "faulthandler.h"
#include "log.h"
#include "imu.h"
#include "pwmwrite.h"
#include "pid.h"
#include "ibus.h"
#include "gps.h"
#include "imu.h"
#include "timer.h"
#include "utils.h"

#include "mpu6050.h"
#include "ms5611.h"
#include "hmc5883.h"
#include "interrupthandler.h"
#include "sensordetect.h"
#include "blackbox.h"
#include "compass.h"

// Loop init variable
#define LOOP_FEQ 200
uint32_t max_excution_time_us;
uint32_t num_tasks;
uint16_t loop_us;


// Loop init variable

/*
 * Test function blink led pc13
 */
static void tongepin()
{
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

task_t task[]={ 
  // {ahrs_update,      0,0,0,1}, /*  imu task 250 hz*/
	
  //{pidUpdate,       0,0,0,1},/*  pid task  250 hz*/

   //{hmc_get_raw,         0,0,0,3},/*  pid task  250/3 hz*/
  //{ms5611_start,         0,0,0,1},
  //{hmc_get_raw,         0,0,0,10},   /*  esc task  250 hz*/
  {tongepin,         0,0,0,100} /*  esc task  250/250 hz*/
  //{HITL_thread,         0,0,0,25}/*  receiver task  50 hz*/
};

void init_scheduler(){
  // init backbox for logging data
  if(black_box_init()){
      // init error
      //HAL_GPIO_WritePin(GPIOX,GPIO_PIN_X,SET);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET);

   
   timer_start(&htim4); // hardwave init
   initPWM(&htim3);    // for pwm
  


  /*------sensor init----------*/ 
  mpu6050_init(&hi2c2); 
  i2cDectect(&hi2c2); // mpu configure i2c bus pass through in oder to connect with hmc5883 
  //delay_ms(2000);     // wait a second after connected battery 
  imuCalibrate();     // calibrate mpu6050 
  //fault_pc13_blink();
  //compassInit();
  ms5611_init(&hi2c2); // inti ms5611
  //gpsInit(&huart1,57600);


  /* scheduler parameters */
	num_tasks = zeroSet();
    max_excution_time_us = zeroSet();
	num_tasks  = sizeof(task)/sizeof(task_t);
	loop_us = (1.0f/ LOOP_FEQ)*1e+6 ;
}

void start_scheduler() {
  static int counter = 0;
  static uint32_t timeUs;
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
  if(counter ==  LOOP_FEQ)counter = 0;
  while((int)(micros() - timeUs) < loop_us);
  timeUs = micros();
}

