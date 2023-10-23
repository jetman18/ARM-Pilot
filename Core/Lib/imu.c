#include "log.h"
#include "imu.h"
#include "stm32f1xx_hal.h"
#include "maths.h"
#include "math.h"
#include "filter.h"
#include "timer.h"
#include "axis.h"
#include "../Driver/mpu6500.h"
#include "../Driver/qmc5883.h"
//#define SPI
#define ACCSMOOTH
#define OFFSET_CYCLE    1000
#define USE_MAG 0

attitude_t AHRS;
float integralFBx;
float integralFBy;
float integralFBz;
float acc_Eframe[3];
float accex,accey,accez;
static float q0=1,q1,q2,q3;
static const float Ki = 0;
static const float Kp = 20;
const float Dt_ahrs = 0.002f;
float dcm[3][3];
static axis3_t acce;
static faxis3_t gyr;
static axis3_t mag;
float acc_pitch;
float acc_roll;

int16_t gyr_offs_x;
int16_t gyr_offs_y;
int16_t gyr_offs_z;
attitude_t quad_;

//IMU configuration parameters
imu_config_t config =
{
  .gyro_f_cut =100,
  .acc_f_cut = 100,
  .cpl_gain = 0.0001f,
  .gyro_slew_threshold=0,
  .acc_slew_threshold=0,
  .dt = 2000, //us
  .gyr_lsb = 32.8f
};

void normalizeV(axis3_t *src)
{
    int32_t sum;
	sum = src->x*src->x + src->y*src->y + src->z*src->z;
    int32_t length = sqrt(sum);
    if (length != 0) {
        src->x = src->x / length;
        src->y = src->y / length;
        src->z = src->z / length;
    }
}
// gyro read and calibrate
void gyro_read(faxis3_t *angle){
	axis3_t p;
	static float gyro_v[3];
	if(gyro_read_raw(&p)){
		return;
	}
	float RC = 1.0f / (2 *M_PIf *config.gyro_f_cut);
    float temp = (float)config.dt*(1e-06f);
	float gain_lpf =temp / (RC + temp);


	float x_  = ((float)(p.x))/config.gyr_lsb;
	float y_ =  ((float)(p.y))/config.gyr_lsb;
	float z_  = ((float)(p.z))/config.gyr_lsb;

    gyro_v[X] = gyro_v[X] + gain_lpf*(x_ - gyro_v[X]);
    gyro_v[Y] = gyro_v[Y] + gain_lpf*(y_ - gyro_v[Y]);
    gyro_v[Z] = gyro_v[Z] + gain_lpf*(z_ - gyro_v[Z]);

    angle->x = gyro_v[X];
    angle->y = gyro_v[Y];
    angle->z = gyro_v[Z];
}
static int32_t store_gyro[3];
void gyro_zero_offset(){
	axis3_t gyro_;
	int16_t count_ = 0;
	for(int i=0;i<OFFSET_CYCLE;i++){
		if(!gyro_read_raw(&gyro_)){
			count_++;
			store_gyro[X] += gyro_.x;
	    	store_gyro[Y] += gyro_.y;
	    	store_gyro[Z] += gyro_.z;
		}
		delay_ms(1);
	}

    if(count_ != 0){
      gyr_offs_x = store_gyro[X]/count_;
      gyr_offs_y = store_gyro[Y]/count_;
      gyr_offs_z = store_gyro[Z]/count_;
    }
}

void mpu_init()
{
   mpu6500_init();
   HAL_Delay(3000);
   gyro_zero_offset();
}

void rotateB2E(faxis3_t *vector,faxis3_t delta)
{
    float mat[3][3];
    float cosx, sinx, cosy, siny, cosz, sinz;
    float coszcosx, sinzcosx, coszsinx, sinzsinx;
	faxis3_t vec;
    
	float temp = config.dt*(float)(1e-06f)*RAD;
	float angleX = delta.x*temp;
	float angleY = delta.y*temp;
	float angleZ = delta.z*temp;

    cosx = cos_approx(angleX);
    sinx = sin_approx(angleX);
    cosy = cos_approx(angleY);
    siny = sin_approx(angleY);
    cosz = cos_approx(angleZ);
    sinz = sin_approx(angleZ);

    coszcosx = cosz * cosx;
    sinzcosx = sinz * cosx;
    coszsinx = sinx * cosz;
    sinzsinx = sinx * sinz;

    mat[0][0] = cosz * cosy;
    mat[0][1] = -cosy * sinz;
    mat[0][2] = siny;
    mat[1][0] = sinzcosx + (coszsinx * siny);
    mat[1][1] = coszcosx - (sinzsinx * siny);
    mat[1][2] = -sinx * cosy;
    mat[2][0] = (sinzsinx) - (coszcosx * siny);
    mat[2][1] = (coszsinx) + (sinzcosx * siny);
    mat[2][2] = cosy * cosx;

    vec.x = vector->x * mat[0][0] + vector->y * mat[1][0] + vector->z * mat[2][0];
    vec.y = vector->x * mat[0][1] + vector->y * mat[1][1] + vector->z * mat[2][1];
    vec.z = vector->x * mat[0][2] + vector->y * mat[1][2] + vector->z * mat[2][2];
   
    vector->x =  vec.x;
	vector->y =  vec.y;
	vector->z =  vec.z;
}
void get_Acc_Angle(attitude_t *m)
{
	axis3_t  acce;
	faxis3_t acc;
	uint32_t sum;
	float length;
    acc_read_raw(&acce);
	sum = acce.x*acce.x + acce.y*acce.y + acce.z*acce.z;
	if(sum == 0){
		return;
	}
	length = invSqrt_((float)sum);
    acc.x = acce.x*length;
    acc.y = acce.y*length;
    acc.z = acce.z*length;
	m->pitch  = atan2_approx(acc.y,acc.z)*180/M_PIf;
	m->roll   = atan2_approx(-acc.x, (1/invSqrt_(acc.y * acc.y + acc.z * acc.z)))*180/M_PIf;
}
void ahrs_update(){
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
    float gx,gy,gz;
	float acc_Bframe[3];
	float hx,hy,bx,bz;
    float wx,wy,wz,mx,my,mz;
	float emx,emy,emz;

    gyro_read(&gyr);
	gx = gyr.x * RAD;
	gy = gyr.y * RAD;
	gz = gyr.z * RAD;

	acc_read_raw(&acce);
	acc_Bframe[X] = (float)acce.x;
	acc_Bframe[Y] = (float)acce.y;
	acc_Bframe[Z] = (float)acce.z;

	if(!((acce.x == 0) && (acce.y == 0) && (acce.z == 0))) {
		norm = invSqrt_(acce.x * acce.x + acce.y * acce.y + acce.z * acce.z);
		accex = acce.x * norm;
		accey = acce.y * norm;
		accez = acce.z * norm;

		acc_pitch  =  atan2_approx(-accey,accez)*DEG;
		acc_roll    = atan2_approx(-accex, (1/invSqrt_(accey * accey + accez * accez)))*DEG;

        if(USE_MAG && qmc_read_raw(&mag)){
			norm = invSqrt_(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
			mx = mag.x * norm;
			my = mag.y * norm;
			mz = mag.z * norm;

			hx = mx * dcm[0][0] + my * dcm[1][0] + mz * dcm[2][0];
			hy = mx * dcm[0][1] + my * dcm[1][1] + mz * dcm[2][1];
			bz = mx * dcm[0][2] + my * dcm[1][2] + mz * dcm[2][2];
			bx = sqrtf(hx * hx + hy * hy);

			wx = bx * dcm[0][0] + bz * dcm[0][2];
			wy = bx * dcm[1][0] + bz * dcm[1][2];
			wz = bx * dcm[2][0] + bz * dcm[2][2];

			emx = my * wz - mz * wy;
			emy = mz * wx - mx * wz;
			emz = mx * wy - my * wx;
		}
		else{
			emx = 0.0f;
			emy = 0.0f;
			emz = 0.0f;
		}
		vx = dcm[0][2];
		vy = dcm[1][2];
		vz = dcm[2][2];

		ex = (accey * vz - accez * vy) + emx;
		ey = (accez * vx - accex * vz) + emy;
		ez = (accex * vy - accey * vx) + emz;

		if(Ki > 0.0f) {
			integralFBx += Ki * ex * Dt_ahrs;
			integralFBy += Ki * ey * Dt_ahrs;
			integralFBz += Ki * ez * Dt_ahrs;
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		gx += Kp * ex;
		gy += Kp * ey;
		gz += Kp * ez;
	}

	gx *= (0.5f * Dt_ahrs);
	gy *= (0.5f * Dt_ahrs);
	gz *= (0.5f * Dt_ahrs);

	q0 += (-q1 * gx - q2 * gy - q3 * gz);
	q1 += ( q0 * gx + q2 * gz - q3 * gy);
	q2 += ( q0 * gy - q1 * gz + q3 * gx);
	q3 += ( q0 * gz + q1 * gy - q2 * gx);

	norm = invSqrt_(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;

	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
    // Quaternion to Rotation matrix
    dcm[0][0] = 2.0f*(0.5f - q2q2  - q3q3);
    dcm[1][0] = 2.0f*(q1q2 - q0q3);
    dcm[2][0] = 2.0f*(q1q3 + q0q2);
    dcm[0][1] = 2.0f*(q1q2 + q0q3);
    dcm[1][1] = 2.0f*(0.5f - q1q1 - q3q3);
    dcm[2][1] = 2.0f*(q2q3 - q0q1);
    dcm[0][2] = 2.0f*(q1q3 - q0q2);
    dcm[1][2] = 2.0f*(q2q3 + q0q1);
    dcm[2][2] = 2.0f*(0.5f - q1q1 - q2q2);
    // Rotate acceleration from Body frame to earth frame
	acc_Eframe[X] = dcm[0][0]*acc_Bframe[X] + dcm[1][0]*acc_Bframe[Y] + dcm[2][0]*acc_Bframe[Z];
	acc_Eframe[Y] = dcm[0][1]*acc_Bframe[X] + dcm[1][1]*acc_Bframe[Y] + dcm[2][1]*acc_Bframe[Z];
	acc_Eframe[Z] = dcm[0][2]*acc_Bframe[X] + dcm[1][2]*acc_Bframe[Y] + dcm[2][2]*acc_Bframe[Z];
	acc_Eframe[X] = acc_Eframe[X]/16384.0f*9.53f;
	acc_Eframe[Y] = acc_Eframe[Y]/16384.0f*9.53f;
	acc_Eframe[Z] = acc_Eframe[Z]/16384.0f*9.53f - 9.82f;
    // Quaternion to euler angle    // deg
	AHRS.roll  = atan2_approx(-dcm[0][2],sqrtf(1 - dcm[0][2]*dcm[0][2])) * DEG;
	AHRS.pitch = atan2_approx(-dcm[1][2],dcm[2][2]) * DEG;
	AHRS.yaw   = atan2_approx(dcm[0][1],dcm[0][0]) * DEG;
	AHRS.acc_x = fapplyDeadband(acc_Eframe[X],0.05);  // dead band 0.05 m/ss
    AHRS.acc_y = fapplyDeadband(acc_Eframe[Y],0.05);
    AHRS.acc_z = fapplyDeadband(acc_Eframe[Z],0.05);
	AHRS.roll_velocity  = gyr.x;    // deg/s
	AHRS.pitch_velocity = gyr.y;
	AHRS.yaw_velocity   = gyr.z;

}

