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
uint16_t prev_wallDistance;
int prevError;
int errorChange;
int fusedErrorChange;

//int activeDistance = 150; //distance where the algorithm activates
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
		errorVelocityI = 0;
		rollAdjustment = 0;
	}

}

int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old, const float vel_acc){

	int32_t result = 0;
	int error = 0;
	int32_t setVel;

	/* Best settings so far
	 * int P = 0;
	 * int I = 20;
	 * int D = 10;
	 *
	 */

	//TUNE THESE
	int P_dist = 100;


	int P_vel = 15;
	//	int I_vel = 60;
	int D_vel = 20;


	//limit measurements for testing
	wallDistance = constrain(wallDistance, -300,300);


	//account for erroneous sensor readings when too close to the wall
	if (wallDistance <= 0){
		wallDistance = 1;
	}

	if (wallDistance - prev_wallDistance > 100){
		wallDistance = prev_wallDistance;
	}

	//Calculate current error
	error = wallDistance - targetDistance;
	error = applyDeadband(error,3); //remove some error measurements < 3

	DEBUG_SET(DEBUG_ESC_SENSOR, 0, wallDistance);

	//Calculate previous error (from previous measurement)
	prevError = prev_wallDistance - targetDistance;
	errorChange = prevError - error;

	//get rid of some noise when stationary
	if (errorChange <= 2 && errorChange >= -2){
		errorChange = 0;
	}

	//add acc data to errorChange (combines both acc and leddar readings)
	fusedErrorChange = errorChange -(accY_old + accY_tmp)/100;

	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, wallDistance);

	setVel = constrain(P_dist * error / 100, -200,+200);

	//just in case it spikes up like crazy (end of wall, etc.)
	if (error > 200){
		error = 0;
	}

	//DEBUG_SET(DEBUG_ESC_SENSOR, 1, setVel);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 2, (accY_old + accY_tmp)/100);

	//DEBUG_SET(DEBUG_ESC_SENSOR, 0, vel_tmp);

	error = setVel - vel_tmp;
	//error = setVel + (accY_old + accY_tmp)/100;
	//error = setVel;

//////////////////////////// P I D //////////////////////////////

	// P
	result = constrain((P_vel * error / 100), -80, +80);
	DEBUG_SET(DEBUG_ESC_SENSOR, 1, result);

	// I
	//errorVelocityI += (error/I_vel);
	//errorVelocityI = constrain(errorVelocityI, -4, +4);
	//	result += errorVelocityI;     // I in range +/-200

	// D
	result -= constrain(D_vel * fusedErrorChange/10, -40, +40);

	//result -= constrain(D_vel * (accY_tmp + accY_old)/512, -40, 40);
	DEBUG_SET(DEBUG_ESC_SENSOR, 2, D_vel * fusedErrorChange/10);
	DEBUG_SET(DEBUG_ESC_SENSOR, 3, 10 * vel_tmp);

	return result;

}

void calculateEstimatedWall(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	//static timeUs_t previousTimeUs = 0;
	//const uint32_t dTime = currentTimeUs - previousTimeUs;
	//previousTimeUs = currentTimeUs;

	static float vel = 0.0f;
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

		// Integrator - Altitude in cm
		//accAlt += (vel_acc * 0.5f) * dt + vel * dt;  // integrate velocity to get distance (x= a/2 * t^2)
		//vel += vel_acc;
		vel = vel_acc;
		//estimatedAltitude = accAlt;


	}
#endif
	// Altitude task is always active when LEDDAR feature is enabled,
	// so we don't need to reset the imu twice (makes for some nasty acc data!)
	//	imuResetAccelerationSum();

	//get leddar distance reading
	wallDistance = ABS(getLeddarWall());
	prev_wallDistance = ABS(getPreviousLeddarWall());

	int32_t vel_tmp = lrintf(vel); //convert from float to long int
	//int32_t vel_tmp = 0;
	static float accY_old = 0.0f;

	rollAdjustment = calculateWallAdjustment(vel_tmp, accY_tmp, accY_old, vel_acc);

	accY_old = accY_tmp;

}

void wallFollow(void){
	//algorithm will only activate when an object comes within activeDistance of the right side of the craft
	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, wallDistance);
	//	if (wallDistance <= activeDistance){

	//calculateWallThrottleAdjustment();
	rcCommand[ROLL] = constrain(rollAdjustment, -60, +60); //limit roll to [-50;+50]
	//	}else{
	//		rollAdjustment = 0;
	//	}
}

