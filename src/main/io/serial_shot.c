/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

//#ifdef USE_SERIALSHOT

#include "io/serial_shot.h"
#include "io/serial.h"

#include "drivers/pwm_output.h"

static serialPort_t *serialShotPort = NULL;

static volatile uint8_t rxBuffer[ESC_DATA_FRAME_SIZE];
static volatile uint8_t rxCnt = 0;
static volatile escData_t escData;
static uint8_t txBuffer[THROTTLE_DATA_FRAME_SIZE];
static uint8_t escCommand[4];

// Receive ISR callback
static void serialShotReceive(uint16_t c, void *data)
{
    UNUSED(data);

    if (((uint8_t)c == 0x88) && (rxBuffer[0] != 0x88))
    {
        rxCnt = 1;
        rxBuffer[0] = 0x88;
    }
    else
    {
        rxBuffer[rxCnt++] = (uint8_t)c;
        
        if (rxCnt == ESC_DATA_FRAME_SIZE)
        {
            rxCnt = 0;
            rxBuffer[0] = 0;
            
            escData.temperature = rxBuffer[1];
            escData.voltage     = ((uint16_t)rxBuffer[2] << 8) | rxBuffer[3];
            escData.current     = ((uint16_t)rxBuffer[4] << 8) | rxBuffer[5];
            escData.consumption = ((uint16_t)rxBuffer[6] << 8) | rxBuffer[7];
            for (uint8_t i = 0; i < 4; i++)
            {
                escData.erpm[i] = ((uint16_t)rxBuffer[8 + 2*i] << 8) | rxBuffer[9 + 2*i];
            }    
        }
    }
}

uint8_t serialShotGetTemperature(void)
{
    return escData.temperature;
}

uint16_t serialShotGetVoltage(void)
{
    return escData.voltage;
}

uint16_t serialShotGetCurrent(void)
{
    return escData.current;
}

uint16_t serialShotGetConsumption(void)
{
    return escData.consumption;
}

uint16_t serialShotGetAverageErpm(void)
{
    return (escData.erpm[0] + escData.erpm[1] + escData.erpm[2] + escData.erpm[3]) / 4;
}

void serialShotMeterReset(void)
{
    escData.voltage = 0;
    escData.current = 0;
    escData.temperature = 0;
    escData.consumption = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        escData.erpm[i] = 0;
    }
}

void serialShotWriteCommand(uint8_t motorIndex, serialShotCommands_e command)
{
    if (motorIndex == ALL_MOTORS) {
        for (uint8_t i = 0; i < 4; i++) {
            escCommand[i] = command;
        }    
    }
    else {
        escCommand[motorIndex] = command;
    }    
}

bool serialShotInit(void)
{
    serialPortConfig_t *portConfig = serialFindPortConfiguration(SERIALSHOT_UART);
    if (portConfig) {
        portConfig->functionMask = FUNCTION_SERIALSHOT;
    }

    if (portConfig) {
        portOptions_e portOptions = 0;

        portOptions = SERIAL_UNIDIR;

        rxCnt = 0;
        rxBuffer[0] = 0;

        serialShotPort = openSerialPort(portConfig->identifier, FUNCTION_SERIALSHOT, serialShotReceive, NULL, \
                                        SERIALSHOT_UART_BAUD, MODE_RXTX, portOptions);
    }

    if (!serialShotPort) {
        return false;
    }

    return true;
}

FAST_CODE void pwmWriteSerialShot(uint8_t index, float value)
{
    uint8_t n = 2 * index;
    uint16_t d = lrintf(value);
    static uint8_t sum = 0;

    d += escCommand[index];
    escCommand[index] = 0;
    
    txBuffer[n] = (uint8_t)(d >> 8);
    txBuffer[n + 1] = (uint8_t)d; 
    
    sum = (uint8_t)(sum + txBuffer[n]);
    sum = (uint8_t)(sum + txBuffer[n + 1]);
    
    if (index == 3)
    {
        txBuffer[8] = sum;
        sum = 0;
        serialWriteBuf(serialShotPort, txBuffer, THROTTLE_DATA_FRAME_SIZE);
    }
}

//#endif

