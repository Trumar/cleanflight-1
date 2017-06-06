/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

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
//#include "drivers/serial.h"
//#include "drivers/serial_uart.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"


serialPort_t *leddarSensorPort;

void leddarInit(void)
{
    portOptions_t options = SERIAL_PARITY_NO | SERIAL_STOPBITS_1;// | SERIAL_NOT_INVERTED;
    portMode_t mode = MODE_RXTX;

    // Initialize serial port
    leddarSensorPort = openSerialPort(SERIAL_PORT_USART2, FUNCTION_LEDDAR, NULL, 115200, mode, options);

    serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(SERIAL_PORT_USART2);
}

void leddarUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    //static int32_t calculatedAltitude = 0;
    uint8_t input[] = {0x01, 0x04, 0x00, 0x14, 0x00, 0x0a, 0x30, 0x09};

    /*Write the input bytes to the LEDDAR sensor to retrieve samples
     * This does not work on UART3 for some reason...*/
    int i;
    for (i = 0; i < 8; i++){
    	serialWrite(leddarSensorPort, input[i]);
    }

    //LED0_TOGGLE;
}

