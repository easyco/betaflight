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

#define USE_TARGET_CONFIG

#define TARGET_BOARD_IDENTIFIER "M765"
#define USBD_PRODUCT_STRING  "MatekF765"

#define ENABLE_DSHOT_DMAR       true

#define LED0_PIN                PD10  //Blue
#define LED1_PIN                PD11  //Green

#define USE_BEEPER
#define BEEPER_PIN              PE2
#define BEEPER_INVERTED

// *************** SPI1 & SPI3, Gyro & ACC *******************

#define USE_SPI
#define USE_SPI_DEVICE_1        //DMA2-2, 2-5
#define GYRO_1_SPI_INSTANCE     SPI1  
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7
#define GYRO_1_CS_PIN           PA4

#define USE_SPI_DEVICE_3        //DMA1-0, 1-5
#define GYRO_2_SPI_INSTANCE     SPI3
#define SPI3_SCK_PIN            PB3
#define SPI3_MISO_PIN           PB4
#define SPI3_MOSI_PIN           PB5
#define GYRO_2_CS_PIN           PD7

#define USE_EXTI
#define MPU_INT_EXTI
#define GYRO_1_EXTI_PIN         PC4
#define GYRO_2_EXTI_PIN         PD4

#define USE_DUAL_GYRO
#define GYRO_CONFIG_USE_GYRO_DEFAULT GYRO_CONFIG_USE_GYRO_BOTH
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW

#define USE_GYRO
#define USE_GYRO_SPI_MPU6500

#define USE_ACC
#define USE_ACC_SPI_MPU6500

#define GYRO_MPU6500_1_ALIGN    CW0_DEG
#define ACC_MPU6500_1_ALIGN     CW0_DEG
#define GYRO_MPU6500_2_ALIGN    CW270_DEG_FLIP
#define ACC_MPU6500_2_ALIGN     CW270_DEG_FLIP

#define GYRO_1_ALIGN            GYRO_MPU6500_1_ALIGN
#define GYRO_2_ALIGN            GYRO_MPU6500_2_ALIGN
#define ACC_1_ALIGN             ACC_MPU6500_1_ALIGN
#define ACC_2_ALIGN             ACC_MPU6500_2_ALIGN

// *************** I2C /Baro/Mag *********************

#define USE_I2C

#define USE_I2C_DEVICE_1
#define I2C_DEVICE_1            (I2CDEV_1)
#define I2C1_SCL                PB6
#define I2C1_SDA                PB7

#define USE_I2C_DEVICE_2
#define I2C_DEVICE_2            (I2CDEV_2)
#define I2C2_SCL                PB10
#define I2C2_SDA                PB11

#define BARO_I2C_INSTANCE       (I2CDEV_2)
#define USE_BARO
#define USE_BARO_BMP280
#define USE_BARO_MS5611
#define USE_BARO_BMP085

#define MAG_I2C_INSTANCE        (I2CDEV_1)
#define USE_MAG
#define USE_MAG_HMC5883
#define USE_MAG_QMC5883
#define USE_MAG_LIS3MDL

// *************** SPI2 OSD ***********************

#define USE_SPI_DEVICE_2
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_MAX7456
#define MAX7456_SPI_INSTANCE    SPI2
#define MAX7456_SPI_CS_PIN      PB12
#define MAX7456_SPI_CLK         (SPI_CLOCK_STANDARD) // 10MHz
#define MAX7456_RESTORE_CLK     (SPI_CLOCK_FAST)

// *************** SPI4 ***************************

#define USE_SPI_DEVICE_4        //DMA2-0, 2-1
#define SPI4_NSS_PIN            PE11
#define SPI4_SCK_PIN            PE12
#define SPI4_MISO_PIN           PE13
#define SPI4_MOSI_PIN           PE14

// *************** SDIO BLACKBOX*******************

#define USE_SDCARD
#define USE_SDCARD_SDIO
#define SDIO_DMA                DMA2_Stream6
#define SDCARD_SPI_CS_PIN       NONE
#define SDCARD_DETECT_PIN       NONE

#define ENABLE_BLACKBOX_LOGGING_ON_SDCARD_BY_DEFAULT

// *************** UART *****************************
#define USE_VCP
#define USB_DETECT_PIN          PA15
#define USE_USB_DETECT

#define USE_UART1
#define UART1_TX_PIN            PA9
#define UART1_RX_PIN            PA10

#define USE_UART2
#define UART2_TX_PIN            PD5
#define UART2_RX_PIN            PD6

#define USE_UART3
#define UART3_TX_PIN            PD8  //DMA1-1
#define UART3_RX_PIN            PD9  //DMA1-3

#define USE_UART4
#define UART4_TX_PIN            PD1
#define UART4_RX_PIN            PD0

#define USE_UART5
#define UART5_TX_PIN            PB9
#define UART5_RX_PIN            PB8

#define USE_UART6
#define UART6_TX_PIN            PC6
#define UART6_RX_PIN            PC7

#define USE_UART7
#define UART7_TX_PIN            PE8
#define UART7_RX_PIN            PE7

#define USE_UART8
#define UART8_TX_PIN            PE1
#define UART8_RX_PIN            PE0
     
//#define USE_SOFTSERIAL1

#define SERIAL_PORT_COUNT       9

#define DEFAULT_RX_FEATURE      FEATURE_RX_SERIAL
#define SERIALRX_PROVIDER       SERIALRX_SBUS
#define SERIALRX_UART           SERIAL_PORT_USART2

#define USE_USARTSHOT
#define SERIALESC_UART          SERIAL_PORT_USART1
#define USE_UART1_TX_DMA
// *************** ADC *****************************
#define USE_ADC
#define ADC1_DMA_STREAM         DMA2_Stream4
#define CURRENT_METER_ADC_PIN   PC3
#define VBAT_ADC_PIN            PC2
#define RSSI_ADC_PIN            PC1
#define EXTERNAL1_ADC_PIN       PC0

#define USE_PINIO
#define PINIO1_PIN              PB2
#define PINIO2_PIN              PC5
#define USE_PINIOBOX

#define DEFAULT_FEATURES        (FEATURE_OSD | FEATURE_TELEMETRY )
#define DEFAULT_VOLTAGE_METER_SOURCE VOLTAGE_METER_ADC
#define DEFAULT_CURRENT_METER_SOURCE CURRENT_METER_ADC

#define USE_ESCSERIAL
#define USE_SERIAL_4WAY_BLHELI_INTERFACE

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff

#define USABLE_TIMER_CHANNEL_COUNT 17
#define USED_TIMERS    (TIM_N(1)|TIM_N(2)|TIM_N(3)|TIM_N(4)|TIM_N(8)|TIM_N(9)|TIM_N(10)|TIM_N(11))
