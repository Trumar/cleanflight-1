#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/axis.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/system.h"

#include "flight/imu.h"

#include "fc/runtime_config.h"
#include "fc/fc_rc.h"
#include "fc/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/leddar.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"

static int32_t rollAdjustment = 0;
static int32_t errorVelocityI = 0;
uint16_t wallDistance;
int activeDistance = 150; //distance where the algorithm activates
uint16_t targetDistance; //target distance from center of quad to wall

int error = 0;
int correction = 0;

void updateWallFollowState(void){
	if (!IS_RC_MODE_ACTIVE(BOXWALL)) {
		DISABLE_FLIGHT_MODE(WALL_MODE);
		return;
	}

	if (!FLIGHT_MODE(WALL_MODE)) {
		ENABLE_FLIGHT_MODE(WALL_MODE);
		targetDistance = wallDistance;
		///initialThrottleHold = rcData[THROTTLE];
		errorVelocityI = 0;
		rollAdjustment = 0;
	}

}

int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old, const float vel_acc){

	int32_t result = 0;
	int error;
	int32_t setVel;

	//TUNE THESE
	//int P_dist = 20;
	int P = 60;
	int I = 0;
	int D = 13;

	DEBUG_SET(DEBUG_ESC_SENSOR, 0, targetDistance);
	DEBUG_SET(DEBUG_ESC_SENSOR, 1, wallDistance);
	error = wallDistance - targetDistance;
	error = applyDeadband(error,1); //remove some error measurements < 1

	setVel = constrain(error, -200,+200); //setVel is error measurement

	error = setVel;// - vel_tmp;


	// P
	result = constrain((P * error / 32), -80, +80);
	DEBUG_SET(DEBUG_ESC_SENSOR, 2, result);

	// I
	errorVelocityI += ((setVel-vel_tmp)/I);
	errorVelocityI = constrain(errorVelocityI, -15, +15);
	//result += errorVelocityI;     // I in range +/-200

	// D

	result += constrain(vel_acc, -10, +10);  //1024 is max acc reading
	//result -= constrain(D/10 *(accY_tmp + accY_old) / 512, -10, +10);  //1024 is max acc reading
	DEBUG_SET(DEBUG_ESC_SENSOR, 3, result);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 2, D * (accY_tmp + accY_old) / 512);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, result);
	return result;

}

void calculateEstimatedWall(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	//static timeUs_t previousTimeUs = 0;
	//const uint32_t dTime = currentTimeUs - previousTimeUs;
	//previousTimeUs = currentTimeUs;

	//static float vel = 0.0f;
	//static float accAlt = 0.0f;

	float accY_tmp = 0;
	const float vel_acc = 0;

#ifdef ACC
	if (sensors(SENSOR_ACC)) {
		//const float dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

		// Integrator - velocity, cm/sec
		if (accSumCount) {
			accY_tmp = (float)accSum[1] / accSumCount; //Y axis
		}
		const float vel_acc = accY_tmp * accVelScale * (float)accTimeSum;

		/*
		// Integrator - Altitude in cm
		accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
		vel += vel_acc;
		//estimatedAltitude = accAlt;
		 */

	}
#endif
	imuResetAccelerationSum();

	//get leddar distance reading
	wallDistance = ABS(getLeddarWall());

	//int32_t vel_tmp = lrintf(vel); //convert from float to long int
	int32_t vel_tmp = 0;
	static float accY_old = 0.0f;

	rollAdjustment = calculateWallAdjustment(vel_tmp, accY_tmp, accY_old, vel_acc);

	accY_old = accY_tmp;

}

void wallFollow(void){
	//algorithm will only activate when an object comes within activeDistance of the right side of the craft
	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, wallDistance);
	if (wallDistance <= activeDistance){

		//calculateWallThrottleAdjustment();
		rcCommand[ROLL] = constrain(rollAdjustment, -50, +50); //limit roll to [-50;+50]
	}else{
		rollAdjustment = 0;
	}
}

