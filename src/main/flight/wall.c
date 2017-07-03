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
int activeDistance = 140; //distance where the algorithm activates
int targetDistance = 120; //target distance away from the object
//int bufferRegion = 20; //range from the wall in which the quad will stay, +/- the targetDistance
int error = 0;

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
		error = applyDeadband(error,5); //remove some error measurements <5

		if (wallDistance >= targetDistance){ //too far from wall
			rcCommand[ROLL] = constrain(-error*2, -500, +500); //limit roll to [-500;+500]
		}
		else if (wallDistance < targetDistance) //too close to the wall
		{
			rcCommand[ROLL] = constrain(error*2 , -500, +500);
		}
	}
	//else if (wallDistance < 0){ //negative sensor readings (inaccurate)
	//		rcCommand[ROLL] = constrain(-45 , -500, +500); //too close to wall, get away from it to get accurate reading
	//}

DEBUG_SET(DEBUG_ESC_SENSOR, 0, wallDistance);
DEBUG_SET(DEBUG_ESC_SENSOR, 1, rcCommand[ROLL]);


}


