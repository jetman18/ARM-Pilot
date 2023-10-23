#include "controller.h"
#include "../lib/pid.h"
#include "../lib/imu.h"
#include "../Driver/ibus.h"
#include "../lib/maths.h"
#include "../lib/pwmwrite.h"
#include "../lib/timer.h"

#define MAX_TILT_ANGLE 45  
#define MAX_ANGLULAR_VELOCITY 100 //deg/s
#define MAX_I_VAL  200
#define MS2SEC(x)  (x*0.001)

static uint32_t pid_timer;
uint16_t pitch_cmd,roll_cmd;
uint8_t init_statte = 1;

float pitch_I;
float pitchKp = 1;
float pitchKi = 0;
float pitchKd = 0;
float last_pitch;
float pitch_D;
float lpf_p_gain = 0.1f;
//
float roll_I;
float rollKp = 1;
float rollKi = 0;
float rollKd = 0;
float last_roll;
float roll_D;
float lpf_r_gain = 0.1f;

void attitudeCrtlReset();
static float antiwindupIntergal(float val);
// ppi controller
void pitchRollControl(float target_roll,float target_pitch){
    if(init_statte){
        attitudeCrtlReset();
        pid_timer = millis();
        last_pitch = AHRS.pitch;
        last_roll = AHRS.roll;
        init_statte = 0;
        return;
    }
    uint32_t deltaT = millis() - pid_timer;
    pid_timer = millis();
    //pitch
    float pitch_error_p = target_pitch - AHRS.pitch;
    float pitch_P = pitch_error_p*pitchKp;
    pitch_I += pitch_error_p*pitchKi*MS2SEC(deltaT);
    pitch_I = antiwindupIntergal(pitch_I);
    pitch_I = constrainf(pitch_I,-MAX_I_VAL,MAX_I_VAL);
    float Dpitch_ = pitchKd*(AHRS.pitch - last_pitch)/MS2SEC(deltaT);
    last_pitch = AHRS.pitch;
    // low pass filer
    pitch_D += lpf_p_gain*(Dpitch_ - pitch_D);
    float pitch_PID = pitch_P + pitch_I + pitch_D;
    //roll
    float roll_error_p = target_roll- AHRS.roll;
    float roll_P = roll_error_p*rollKp;
    roll_I += roll_error_p*rollKi*MS2SEC(deltaT);
    roll_I = antiwindupIntergal(roll_I);
    roll_I = constrainf(roll_I,-MAX_I_VAL,MAX_I_VAL);
    float Droll_ = rollKd*(AHRS.roll - last_roll)/MS2SEC(deltaT);
    last_roll = AHRS.roll;
    // low pass filer
    roll_D += lpf_p_gain*(Droll_ - roll_D);
    float roll_PID = roll_P + roll_I + roll_D;
    //
    pitch_cmd = 1500  - roll_PID + pitch_PID;
    roll_cmd  = 1500  + roll_PID + pitch_PID;
    pitch_cmd = constrain(pitch_cmd,1000,2000);
    roll_cmd  = constrain(roll_cmd,1000,2000);
    writePwm(0,pitch_cmd);
    writePwm(1,roll_cmd);
}

static float antiwindupIntergal(float val){
     if(abs(val)<1.0f){
        return 0.0f;
     }
     else
        return val;
}
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
void attitudeCrtlReset()
{
   roll_I     = 0;
   roll_D     = 0;
   pitch_D    = 0;
   pitch_I    = 0;
   last_roll  = 0;
   last_pitch = 0;
   pitch_cmd  = 0;
   roll_cmd   = 0;
}
