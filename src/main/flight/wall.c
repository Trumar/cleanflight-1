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
int activeDistance = 100; //distance where the algorithm activates
int targetDistance = 30; //target distance from center of quad to wall

int errorP = 0;
int errorI = 0;
int error = 0;
int correction = 0;
int P = 2;
int I = 20;
int D = 10;

int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old){

	int32_t result = 0;
	int error;
	int32_t setVel;

	int P = 20;
	int I = 10;
	int D = 10;

	error = wallDistance - targetDistance;
	error = applyDeadband(error,3); //remove some error measurements < 3

	setVel = constrain(error ,-500,+500); //setVel is error measurement

	error = setVel - vel_tmp;

	// P
	result = constrain((P * error / 32), -300, +300);
	DEBUG_SET(DEBUG_ESC_SENSOR, 0, result);
	// I
	errorVelocityI += (error/I);
	errorVelocityI = constrain(errorVelocityI, -15, +15);
	result += errorVelocityI;     // I in range +/-200
	DEBUG_SET(DEBUG_ESC_SENSOR, 1, errorVelocityI);

	// D
	result -= constrain(D * (accY_tmp + accY_old) / 512, -50, +50);
	DEBUG_SET(DEBUG_ESC_SENSOR, 2, D * (accY_tmp + accY_old) / 512);

	return result;

}

void calculateEstimatedWall(timeUs_t currentTimeUs)
{
	static timeUs_t previousTimeUs = 0;
	//const uint32_t dTime = currentTimeUs - previousTimeUs;
	previousTimeUs = currentTimeUs;

	static float vel = 0.0f;
	static float accAlt = 0.0f;

	float accY_tmp = 0;

#ifdef ACC
	if (sensors(SENSOR_ACC)) {
		const float dt = accTimeSum * 1e-6f; // delta acc reading time in seconds

		// Integrator - velocity, cm/sec
		if (accSumCount) {
			accY_tmp = (float)accSum[1] / accSumCount; //Y axis
		}
		const float vel_acc = accY_tmp * accVelScale * (float)accTimeSum;

		// Integrator - Altitude in cm
		accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
		vel += vel_acc;
		//estimatedAltitude = accAlt;
	}
#endif
	imuResetAccelerationSum();

	//get leddar distance reading
	wallDistance = ABS(getLeddarWall());

	int32_t vel_tmp = lrintf(vel); //convert from float to long int


	static float accY_old = 0.0f;

	rollAdjustment = calculateWallAdjustment(vel_tmp, accY_tmp, accY_old);

	accY_old = accY_tmp;

}

void wallFollow(void){
	//algorithm will only activate when an object comes within activeDistance of the right side of the craft
	if (wallDistance <= activeDistance){
		DEBUG_SET(DEBUG_ESC_SENSOR, 3, wallDistance);
		//calculateWallThrottleAdjustment();
		rcCommand[ROLL] = constrain(rollAdjustment, -50, +50); //limit roll to [-50;+50]
	}else{
		rollAdjustment = 0;
	}
}



