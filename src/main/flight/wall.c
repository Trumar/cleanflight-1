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

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/system.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/leddar.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"

void leddarWall(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    //wall follow code here

    //LED0_TOGGLE;
}
