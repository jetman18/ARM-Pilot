#include "pid.h"
#include "imu.h"
#include "ibus.h"
#include "maths.h"
#include "pwmwrite.h"
#include "timer.h"
#include "plane.h"
#include "utils.h"

typedef struct pid_attitude{  
    float kpf;
    float kp;
    float ki;
    float kd;
    float I_term;
    float last_rate;
    float f_cut_D;
    float D_filted;
    float rate_limit;
}pid_attitude;

float max_pid_value = 1000;
static float max_tilt_angle = 45;  
static float max_angula_rate = 100;         //deg/s
static float max_intergal_value =  200;
static float Dt = 0.004f;
uint8_t init_statte = 1;

static uint32_t pid_timer;
uint16_t pitch_cmd,roll_cmd;

pid_attitude Roll = {
   1,   //kpf
   1,   //kp
   0.5,   //ki
   0,   //kd
   0,   //I_term
   0,   //last_rate
   30,   //f_cut_D
   0,    //D_filted
   100   //rate limit
};

pid_attitude Pitch = {
   1,   //kpf
   1,   //kp
   0.5,   //ki
   0,   //kd
   0,   //I_term
   0,   //last_rate
   30,   //f_cut_D
   0,    //D_filted
   100   // rate limit
};


/* axis control with ppid controller
 * input set angle
 * return PID value
 */
int32_t roll_control(float target_roll){
   // angle error
   float Delat_angle = (target_roll - AHRS.roll)*Roll.kpf;
   // rate limited
   Delat_angle = constrainf(Delat_angle,-Roll.rate_limit,Roll.rate_limit);

   // rate error
   float Delta_rate_angle = Delat_angle - AHRS.roll_rate;

   // calculate proportional term
   float P_term =  Delta_rate_angle*Roll.kp;

   // calculate derivative term
   float D_temp =  (AHRS.roll_rate - Roll.last_rate)/Dt;
   Roll.last_rate = AHRS.roll_rate;

   // Apply low pass filter 
   float RC = 1.0f / (2 *M_PIf *Roll.f_cut_D);
   float gain_lpf = Dt/(RC + Dt);
   Roll.D_filted += gain_lpf*(D_temp - Roll.D_filted);
   float D_term = Roll.D_filted*Roll.kd;

   // calculate intergal term
   Roll.I_term += Delta_rate_angle*Roll.ki*Dt;
   //float max_i = max_pid_value - P_term - D_term;
   //Roll.I_term = constrainf(Roll.I_term,-max_i,max_i);
   Roll.I_term = constrainf(Roll.I_term,-max_intergal_value,max_intergal_value);
   
   int32_t PID = (int32_t)(P_term  + D_term + Roll.I_term);
   PID = constrain(PID,-max_pid_value,max_pid_value);
   // Return PID value
   return PID;
}

/* axis control with ppid controller
 * input target angle
 * return PID value
 */
int32_t pitch_control(float target_pitch){
   // angle error
   float Delat_angle = (target_pitch - AHRS.pitch)*Pitch.kpf;
   // rate limted
   Delat_angle = constrainf(Delat_angle,-Pitch.rate_limit,Pitch.rate_limit);

   // rate error
   float Delta_rate_angle = Delat_angle - AHRS.pitch_rate;

   // calculate proportional term
   float P_term =  Delta_rate_angle*Pitch.kp;

   // calculate derivative term
   float D_temp =  (AHRS.pitch_rate - Pitch.last_rate)/Dt;
   Pitch.last_rate = AHRS.pitch_rate;
   // Apply low pass filter 
   float RC = 1.0f / (2 *M_PIf *Pitch.f_cut_D);
   float gain_lpf = Dt/(RC + Dt);
   Pitch.D_filted += gain_lpf*(D_temp - Pitch.D_filted);
   float D_term = Pitch.D_filted*Pitch.kd;

   // calculate intergal term
   Pitch.I_term += Delta_rate_angle*Pitch.ki*Dt;
   // float max_i = max_pid_value - P_term - D_term;
   // Pitch.I_term = constrainf( Pitch.I_term,-max_i,max_i);
   Pitch.I_term = constrainf(Pitch.I_term,-max_intergal_value,max_intergal_value);
   
   int32_t PID = (int32_t)(P_term  + D_term + Roll.I_term);
   PID = constrain(PID,-max_pid_value,max_pid_value);
   // Return PID value
   return PID;
}

int32_t altitudeControl(){


}
int32_t speedControl(){

    
}



void heading_control(float yaw_cmd){


}

/*
pitch_cmd = 1500  - roll_PID + pitch_PID;
roll_cmd  = 1500  + roll_PID + pitch_PID;
pitch_cmd = constrain(pitch_cmd,1000,2000);
roll_cmd  = constrain(roll_cmd,1000,2000);
writePwm(0,pitch_cmd);
writePwm(1,roll_cmd);
*/

static float antiwindupIntergal(float val){
     if(fabs(val)<1.0f){
        return 0.0f;
     }
     else
        return val;
}
/*
void setPIDgain(int axis,float kp,float ki,float kd){
    if (axis == 0){
        rollKp = kp;
        rollKi = ki;
        rollKd = kd;
    }
    else if (axis == 1)
    {
        pitchKp = kp;
        pitchKi = ki;
        pitchKd = kd;
    }
}

void pidResetAll()
{
  Pitch = {
   1,   //kpf
   1,   //kp
   0,   //ki
   0,   //kd
   0,   //I_term
   0,   //last_rate
   0,   //f_cut_D
   0,    //D_filted
   0
   };
   ////////////////////////
   Roll = {
   1,   //kpf
   1,   //kp
   0,   //ki
   0,   //kd
   0,   //I_term
   0,   //last_rate
   0,   //f_cut_D
   0,    //D_filted
   0
   };
}
*/