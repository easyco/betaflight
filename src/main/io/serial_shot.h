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
//#if defined (USE_USARTSHOT)

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define ESC_DATA_FRAME_SIZE         16
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
    SERIALSHOT_CMD_MAX = 47
}   serialShotCommands_e;

uint8_t serialShotGetTemperature(void);
uint16_t serialShotGetVoltage(void);
uint16_t serialShotGetCurrent(void);
uint16_t serialShotGetConsumption(void);
uint16_t serialShotGetAverageErpm(void);
void serialShotMeterReset(void);
void serialShotWriteCommand(uint8_t motorIndex, serialShotCommands_e command);

bool serialShotInit(void);
FAST_CODE void pwmWriteSerialShot(uint8_t index, float value);

