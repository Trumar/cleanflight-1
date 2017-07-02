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
#include "drivers/system.h"
//#include "drivers/serial.h"
//#include "drivers/serial_uart.h"

#include "fc/runtime_config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"

#include "drivers/light_led.h"

#include "io/serial.h"


serialPort_t *leddarSensorPort;
serialPort_t *leddarSensorPort2;
uint16_t distance;
uint16_t distance2;

void leddarInit(void)
{
    portOptions_t options = SERIAL_PARITY_NO | SERIAL_STOPBITS_1;// | SERIAL_NOT_INVERTED;

    portMode_t mode = MODE_RXTX;
    portMode_t mode2 = MODE_RX;

    serialPortIdentifier_e port = SERIAL_PORT_USART2;
    serialPortIdentifier_e port2 = SERIAL_PORT_USART3;

    // Initialize serial ports
    leddarSensorPort = openSerialPort(port, FUNCTION_LEDDAR, NULL, 57600, mode, options); //57600 baud rate
    leddarSensorPort2 = openSerialPort(port2, FUNCTION_LEDDAR, NULL, 57600, mode2, options);
    //serialPortUsage_t *serialPortUsage = findSerialPortUsageByIdentifier(SERIAL_PORT_USART2);
}

uint16_t getLeddarAlt(){
	return distance;
}

void leddarUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
    uint8_t receivedByte = 0;//[3] = {0x00, 0x00, 0x00};

    uint8_t sensorSample[25] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint8_t sensorSample2[25] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    distance = 0;

    /* SERIAL WRITE SECTION (SEND DATA TO LEDDAR SENSOR)*/
    uint8_t input[] = {0x01, 0x04, 0x00, 0x14, 0x00, 0x0a, 0x30, 0x09};

    //Write the input bytes to the LEDDAR sensor to retrieve samples
    // This does not work on UART3 for some reason...
    int i;
    for (i = 0; i < 8; i++){
    	serialWrite(leddarSensorPort, input[i]);
    }

    /*SERIAL READ SECTION */
    //Sample format (of interest is 05C2) -> byte locations [11] and [12]
    //01 04 14 C9 D6 00 2D 26 CC 00 01 05 C2 48 87 00 00 00 00 00 00 00 00 CE DA
    //0x01 0x04 0x14 0xC9 0xD6 0x00 0x2D 0x26 0xCC 0x00 0x01 0x05 0xC2 0x48 0x87 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xCE 0xDA
    i = 0;
    while (serialRxBytesWaiting(leddarSensorPort) > 0){
    	receivedByte = serialRead(leddarSensorPort);
    	//serialWrite(leddarSensorPort, receivedByte);

    	sensorSample[i] = receivedByte;
    	i++;
    }

    i = 0;
    while (serialRxBytesWaiting(leddarSensorPort2) > 0){
    	receivedByte = serialRead(leddarSensorPort2);
    	//serialWrite(leddarSensorPort, receivedByte);

    	sensorSample2[i] = receivedByte;
    	i++;
    }


//displays in distance in cm
    distance = sensorSample[12] | sensorSample[11] << 8;
    distance = distance/10;

    //distance = distance + 20; //compensate for 5.5V source
    distance2 = sensorSample2[12] | sensorSample2[11] << 8;
    distance2 = (distance2/10)-380; //380 cm offset for this sensor for whatever reason...

   	debug[0] = distance;
   	debug[1] = distance2;

}

