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

#include "fc/runtime_config.h"
#include "fc/fc_rc.h"
#include "fc/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/leddar.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"

uint16_t wallDistance;
int activeDistance = 90; //distance where the algorithm activates
int targetDistance = 60; //target distance away from the object
//int errorDeadband = 25; //keeps quad within set region, limits oscillation to and from wall
int errorP = 0;
int errorI = 0;
int error = 0;
int correction = 0;
int P = 6;
int I = 10;

void calculateEstimatedWall(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    //wall follow code here
    wallDistance = ABS(getLeddarWall());

}

void wallFollow(void){

	//algorithm will only activate when an object comes within 100 cm of the right side of the craft
	if (wallDistance <= activeDistance){

		error = wallDistance - targetDistance;
		error = applyDeadband(error,3); //remove some error measurements < 3

		//only apply correction if needed
		if (error != 0 ){
			//if (wallDistance >= targetDistance){ //too far from wall
				//P
			correction = constrain(error/P, -10, 10);

			DEBUG_SET(DEBUG_ESC_SENSOR, 1, correction);
		//	}
		//	else if (wallDistance < targetDistance) //too close to the wall
		//	{

		//	}

			//I
			errorI += (error/I);
			errorI = constrain(errorI, -2, +2);
			DEBUG_SET(DEBUG_ESC_SENSOR, 2, errorI);

			correction += errorI;

			rcCommand[ROLL] = constrain(correction, -500, +500); //limit roll to [-500;+500]
		}
	}

DEBUG_SET(DEBUG_ESC_SENSOR, 0, wallDistance);



}


