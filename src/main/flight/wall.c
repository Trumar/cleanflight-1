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
#include "flight/altitude.h"

#include "fc/runtime_config.h"
#include "fc/fc_rc.h"
#include "fc/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/leddar.h"
#include "sensors/battery.h"
#include "sensors/barometer.h"

#include "drivers/light_led.h"

#include "io/serial.h"

static int32_t rollAdjustment = 0;
static int32_t errorVelocityI = 0;
int32_t prev_vel_tmp;
uint16_t wallDistance;
uint16_t prev_wallDistance;
int prevError;
int errorChange;
int fusedErrorChange;

//velocity variables
int32_t vel_tmp;
float acc_tmp;
float acc_old;

uint16_t targetDistance; //target distance from center of quad to wall
int error = 0;
int correction = 0;

void sendVeloctyToWall(int32_t velY_tmp, float accY_tmp, float accY_old){
	vel_tmp = velY_tmp;
	acc_tmp = accY_tmp;
	acc_old = accY_old;
}


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

int32_t calculateWallAdjustment(int32_t vel_tmp, float accY_tmp, float accY_old){
	//you can use the CLI command 'set debug_mode = ESC_SENSOR' to get the debug values for this part

	int32_t result = 0;
	int error = 0;
	int32_t setVel;

	//TUNE THESE
	int P_dist = 100;

	int P_vel = 0;
	//int I_vel = 0;
	int D_vel = 30;

	//limit measurements for testing
	wallDistance = constrain(wallDistance, -300,300);

	//account for erroneous sensor readings when too close to the wall
	if (wallDistance <= 0){
		wallDistance = 1;
	}
	if (wallDistance - prev_wallDistance > 100){
		wallDistance = prev_wallDistance;
	}

	DEBUG_SET(DEBUG_ESC_SENSOR, 0, targetDistance);
	DEBUG_SET(DEBUG_ESC_SENSOR, 1, wallDistance);

	//Calculate current error
	error = targetDistance - wallDistance;
	error = applyDeadband(error,1); //remove some error measurements < 3




	//Calculate previous error (from previous measurement)
	prevError = (prev_vel_tmp) - (P_dist * (targetDistance - prev_wallDistance)/100);

	setVel = constrain(P_dist * error / 100, -200,+200);

	//just in case it spikes up like crazy (end of wall, etc.)
	if (error > 200){
		error = 0;
	}

	DEBUG_SET(DEBUG_ALTITUDE, 3, vel_tmp);

	error = setVel - vel_tmp;

	DEBUG_SET(DEBUG_ESC_SENSOR, 2, error);

	errorChange = constrain(prevError - error, -20, 20);
	//errorChange = constrain(prevError - error, -20, 20);
	//get rid of some noise when stationary
	//	if (errorChange <= 1 && errorChange >= -1){
	//		errorChange = 0;
	//	}

	//add acc data to errorChange (combines both acc and leddar readings)
	//fusedErrorChange = errorChange/2 +(accY_old + accY_tmp)/50;
	//fusedErrorChange = errorChange -(accY_old + accY_tmp)/100;

	//////////////////////////// P I D //////////////////////////////
	// P
	result = constrain((P_vel * error / 100), -100, +100);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 2, result);

	// I
	//errorVelocityI += (error/I_vel);
	//errorVelocityI = constrain(errorVelocityI, -4, +4);
	//result += errorVelocityI;     // I in range +/-200

	// D
	DEBUG_SET(DEBUG_ESC_SENSOR, 3, (accY_old + accY_tmp));

	result -= constrain(D_vel * errorChange/10, -40, +40);
	//result -= constrain(D_vel * (accY_old + accY_tmp)/512, -100, +100);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 1, (accY_old + accY_tmp)/512);


	return result;

}

void calculateEstimatedWall(timeUs_t currentTimeUs)
{
UNUSED(currentTimeUs);

	//get leddar distance readings for wall sensor
#ifdef LEDDAR
	if (sensors(SENSOR_LEDDAR)){
		wallDistance = ABS(getLeddarWall());
		prev_wallDistance = ABS(getPreviousLeddarWall());
	}
#endif

	rollAdjustment = calculateWallAdjustment(vel_tmp, acc_tmp, acc_old);


}

void wallFollow(void){
	//algorithm will only activate when an object comes within activeDistance of the right side of the craft

	//	if (wallDistance <= activeDistance){

	//calculateWallThrottleAdjustment();
	rcCommand[ROLL] = constrain(rollAdjustment, -100, +100); //limit roll to [-50;+50]
	//	}else{
	//		rollAdjustment = 0;
	//	}
}

