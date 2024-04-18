#ifndef __SHOOT__
#define __SHOOT__

unsigned char shoot_ProTerminate(void);
void ShootTask(void const *pvParameters);

void shootMotor_control(void);
	
void pluck_check(void);
void shoot_PID_init(void);
void shoot_mode_check(void);
float imu_filter(float imu_in);

#endif //__SHOOT__


/* end */


