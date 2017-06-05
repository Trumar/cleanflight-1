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

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/io.h"
#include "drivers/light_led.h"
#include "drivers/gpio.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"


static serialPort_t *leddarSensorPort = NULL;

bool leddarInit(void)
{

    serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_BLACKBOX);
    if (!portConfig) {
        return false;
    }

    portOptions_t options = SERIAL_PARITY_NO | SERIAL_STOPBITS_1 | SERIAL_NOT_INVERTED;
    portMode_t mode = MODE_TX;

    // Initialize serial port
    leddarSensorPort = openSerialPort(portConfig->identifier, FUNCTION_BLACKBOX, NULL, 115200, mode, options);

    return leddarSensorPort != NULL;
}


void leddarUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    //static int32_t calculatedAltitude = 0;
    //uint8_t input[] = {0x01, 0x04, 0x00, 0x14, 0x00, 0x0a, 0x30, 0x09};
    static const uint8_t input[] = {0x01};

/* UART3 confirmed to be active point at this stage
    if (leddarSensorPort->identifier == SERIAL_PORT_USART3){
    	LED0_TOGGLE;
    }
    */

    serialWrite(leddarSensorPort, *input);
    LED0_TOGGLE;

    //we want to use UART3, so the id is 2
    /*leddarPort = openSerialPort(SERIAL_PORT_USART3, FUNCTION_RX_SERIAL, NULL,
    	                                         115200, MODE_RXTX,
    											 SERIAL_STOPBITS_1);
*/

}

