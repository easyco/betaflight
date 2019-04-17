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

static uint8_t rxBuffer[MAX_SERIALSHOT_TELEMETRY_FRAME_LENGTH];
static volatile uint8_t rxCnt = 0;
static volatile uint8_t telemetryLen = 0;
static volatile escData_t escData;
static uint8_t txBuffer[THROTTLE_DATA_FRAME_SIZE];
static uint8_t escCommand[4];

static uint8_t getCheckSum(uint8_t *buffer, uint8_t len)
{
    uint8_t sum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        sum = (uint8_t)(sum + buffer[i]);
    }
    return sum;
}

// Receive ISR callback
static void serialShotReceive(uint16_t c, void *data)
{
    UNUSED(data);

    if (rxBuffer[0] == 0) {
        switch (c) {
            case SERIALSHOT_TELEMETRY_ONLY_ERPM:
                telemetryLen = 10;
                rxBuffer[0] = (uint8_t)c;
                rxCnt = 1;
                break;
        
            case SERIALSHOT_TELEMETRY_TEMPERATURE:
                telemetryLen = 11;
                rxBuffer[0] = (uint8_t)c;
                rxCnt = 1;
                break;

            case SERIALSHOT_TELEMETRY_VOLTAGE:
            case SERIALSHOT_TELEMETRY_CURRENT:
            case SERIALSHOT_TELEMETRY_MAH:
                telemetryLen = 12;
                rxBuffer[0] = (uint8_t)c;
                rxCnt = 1;
                break;

            default:
                break;
        }
    }
    else {
        rxBuffer[rxCnt++] = (uint8_t)c;
        
        if (rxCnt < telemetryLen) return;
        if (rxBuffer[rxCnt - 1] != getCheckSum(rxBuffer, telemetryLen - 1)) {
            rxBuffer[0] = 0;
            return;
        }

        for (uint8_t i = 0; i < 4; i++) {
            escData.erpm[i] = ((uint16_t)rxBuffer[1 + 2 * i] << 8) | rxBuffer[2 + 2 * i];
        }

        switch (rxBuffer[0]) {
        case SERIALSHOT_TELEMETRY_TEMPERATURE:
            escData.temperature = rxBuffer[9];
            break;

        case SERIALSHOT_TELEMETRY_VOLTAGE:
            escData.voltage = ((uint16_t)rxBuffer[9] << 8) | rxBuffer[10];
            break;

        case SERIALSHOT_TELEMETRY_CURRENT:
            escData.current = ((uint16_t)rxBuffer[9] << 8) | rxBuffer[10];
            break;

        case SERIALSHOT_TELEMETRY_MAH:
            escData.consumption = ((uint16_t)rxBuffer[9] << 8) | rxBuffer[10];
            break;

        default:
            break;
        }
        rxBuffer[0] = 0;
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

uint16_t serialShotGetErpm(uint8_t motorIndex)
{
    return escData.erpm[motorIndex];
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
        rxCnt = 0;
        rxBuffer[0] = 0;

        serialShotPort = openSerialPort(portConfig->identifier, FUNCTION_SERIALSHOT, serialShotReceive, rxBuffer,
                                        SERIALSHOT_UART_BAUD, MODE_RXTX, SERIAL_UNIDIR);
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

