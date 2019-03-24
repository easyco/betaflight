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

#pragma once

#include <stdint.h>
#include <stdbool.h>

//#define ESC_DATA_FRAME_SIZE         16
#define THROTTLE_DATA_FRAME_SIZE    9
#define SERIALSHOT_UART_BAUD        921600

typedef struct {
    uint8_t     temperature;    // C degrees
    uint16_t    voltage;        // 0.01V
    uint16_t    current;        // 0.01A
    uint16_t    consumption;    // mAh
    uint16_t    erpm[4];        // 0.01erpm    
}   escData_t;

typedef enum {
    SERIALSHOT_CMD_MOTOR_STOP = 0,
    SERIALSHOT_CMD_BEACON1,
    SERIALSHOT_CMD_BEACON2,
    SERIALSHOT_CMD_BEACON3,
    SERIALSHOT_CMD_BEACON4,
    SERIALSHOT_CMD_BEACON5,
    SERIALSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,
    SERIALSHOT_CMD_SPIN_DIRECTION_REVERSED = 21,
    SERIALSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32,
    SERIALSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 33,
    SERIALSHOT_CMD_MAX = 47
}   serialShotCommands_e;

typedef enum {
    SERIALSHOT_TELEMETRY_ONLY_ERPM = 1,
    SERIALSHOT_TELEMETRY_TEMPERATURE = 2,
    SERIALSHOT_TELEMETRY_VOLTAGE = 3,
    SERIALSHOT_TELEMETRY_CURRENT = 4,
    SERIALSHOT_TELEMETRY_MAH = 5,
}   serialShotTelemetryType_e;

#define MAX_SERIALSHOT_TELEMETRY_FRAME_LENGTH       12

uint8_t serialShotGetTemperature(void);
uint16_t serialShotGetVoltage(void);
uint16_t serialShotGetCurrent(void);
uint16_t serialShotGetConsumption(void);
uint16_t serialShotGetAverageErpm(void);
uint16_t serialShotGetErpm(uint8_t motorIndex);
void serialShotWriteCommand(uint8_t motorIndex, serialShotCommands_e command);

bool serialShotInit(void);
FAST_CODE void pwmWriteSerialShot(uint8_t index, float value);

