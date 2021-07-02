#pragma once

#include "ch.hpp"
#include "Board.hpp"
#include "board.h"
#include "hal_serial.h"
#include "LocalMath.hpp"
#include "RobotConf.hpp"

#define LOGGING_DRIVER SD2
#define SHELL_DRIVER LOGGING_DRIVER

#define LOGGING_TX_PIN PAL_LINE(GPIOA, 2U)
#define LOGGING_TX_PIN_MODE PAL_MODE_ALTERNATE(7)
#define LOGGING_RX_PIN PAL_LINE(GPIOA, 3U)
#define LOGGING_RX_PIN_MODE PAL_MODE_ALTERNATE(7)

#define LED_LINE PAL_LINE(GPIOB, 8U)
#define XL320_DRIVER SD1

#define XL320_DATA_PIN      PAL_LINE(GPIOA, 9U)
#define XL320_DATA_PIN_MODE PAL_MODE_ALTERNATE(7)

#define CAN_DRIVER CAND1
#define CAN_TX_PIN      PAL_LINE(GPIOA, 12U)
#define CAN_TX_PIN_MODE PAL_MODE_ALTERNATE(9)
#define CAN_RX_PIN      PAL_LINE(GPIOA, 11U)
#define CAN_RX_PIN_MODE PAL_MODE_ALTERNATE(9)

CANConfig const canConfig = {
        .NBTP = 0x070C01,
        .DBTP = 0,
        .CCCR = 0,
        .TEST = 0,
        .RXGFC = 0
};

#define I2C_SERVO_DRIVER I2CD2

#define I2C_SERVO_SDA_PIN      PAL_LINE(GPIOF, 0U)
#define I2C_SERVO_SDA_PIN_MODE PAL_MODE_ALTERNATE(4)
#define I2C_SERVO_SCL_PIN      PAL_LINE(GPIOB, 7U)
#define I2C_SERVO_SCL_PIN_MODE PAL_MODE_ALTERNATE(4)

I2CConfig const i2cConfig = {
        .timingr = 0x20A0C4DF,
        .cr1 = 0,
        .cr2 = 0,
};



