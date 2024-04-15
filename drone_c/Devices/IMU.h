#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"

typedef struct IMU
{
	float yawAngle;
	float pitchAngle;
	float yawSpeed;
	float pitchSpeed;
	uint32_t sendId;
	uint32_t receiveId;
  uint32_t updateTime;
	uint8_t Ever_Lost;	
} IMUType_def;


#endif // __DBUS_H__


/* end */















