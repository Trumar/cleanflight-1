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
	int P_dist = 140;

	int P_vel = 20;
	//int I_vel = 0;
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
	error = targetDistance - wallDistance;
	setVel = constrain(P_dist * error / 100, -200,+200);

	//Calculate previous error (from previous measurement)
	prevError = (P_dist * (targetDistance - prev_wallDistance)/100);

	errorChange = (targetDistance - prev_wallDistance) - error;


	//just in case it spikes up like crazy (end of wall, etc.)
	if (error > 200){
		error = 0;
	}




	DEBUG_SET(DEBUG_ESC_SENSOR, 0, wallDistance);



	error = vel_tmp - setVel; //error in cm/s
	DEBUG_SET(DEBUG_ESC_SENSOR, 1, error);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 2, error);
	//////////////////////////// P I D //////////////////////////////
	// P
	result = constrain((P_vel * error / 100), -100, +100);

	// I
	//errorVelocityI += (error/I_vel);
	//errorVelocityI = constrain(errorVelocityI, -4, +4);
	//result += errorVelocityI;     // I in range +/-200

	// D
	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, D_vel * (accY_old + accY_tmp)/512);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 3, D_vel * (accY_old + accY_tmp)/512);

	result -= constrain(D_vel * (accY_old + accY_tmp)/512, -40, +40);
	//DEBUG_SET(DEBUG_ESC_SENSOR, 1, (accY_old + accY_tmp)/512);
	DEBUG_SET(DEBUG_ESC_SENSOR, 2, (accY_old + accY_tmp));

	//Basically a better derivative block for the PID controller
	//add a "pulse" ROLL command to stop the quadcopter from moving in the direction it is heading based on the current error and direction
	if(ABS(targetDistance - wallDistance) <= 25){
		if ((accY_old + accY_tmp) >= -50 && (accY_old + accY_tmp) <= -20){
			if (error >= 15 && error <= 50){
				DEBUG_SET(DEBUG_ESC_SENSOR, 3, -10);//coming in towards wall near target
				result -= 40;
				//rcCommand[ROLL] = -20;
			}else if (error >= -50 && error <= -15){
				DEBUG_SET(DEBUG_ESC_SENSOR, 3, 10);//going away from wall, approaching target
				result += 40;
				//rcCommand[ROLL] = 20;
			}
			else{
				DEBUG_SET(DEBUG_ESC_SENSOR, 3, 0);
			}
		}else{ //acceleration check
			DEBUG_SET(DEBUG_ESC_SENSOR, 3, 0);
		}
	}
	else{ //no correction needed, out of range
		DEBUG_SET(DEBUG_ESC_SENSOR, 3, 0)
	}

	return result;

}//calculateWallAdjustment

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

