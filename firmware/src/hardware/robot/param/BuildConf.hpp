#pragma once

#include "ch.hpp"
#include "Board.hpp"
#include "board.h"
#include "hal_serial.h"
#include "LocalMath.hpp"

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

#define I2C_DRIVER I2CD2

#define I2C_SDA_PIN      PAL_LINE(GPIOF, 0U)
#define I2C_SDA_PIN_MODE PAL_MODE_ALTERNATE(4)
#define I2C_SCL_PIN      PAL_LINE(GPIOA, 9U)
#define I2C_SCL_PIN_MODE PAL_MODE_ALTERNATE(4)

I2CConfig const i2cConfig = {
        .timingr = 0x10707DBC,
        .cr1 = 0,
        .cr2 = 0,
};


constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ID  = 13; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_FRONT_LEFT_ID      = 14; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_FRONT_RIGHT_ID     = 15; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ID = 16; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_RIGHT_ID  = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_ID      = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_ID     = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_ID       = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_ID   = 0; //TODO: CHANGE ME!

constexpr uint8_t PLIERS_REAR_FAR_RIGHT_PWM_CHANNEL = 2; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_PWM_CHANNEL = 3; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_PWM_CHANNEL = 1; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_PWM_CHANNEL = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_PWM_CHANNEL = 4; //TODO: CHANGE ME!

constexpr float PLIERS_REAR_FAR_RIGHT_IDLE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_RIGHT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_IDLE_ANGLE = 250; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_ACTIVE_ANGLE = 150; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ANGLE_IDLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!

constexpr uint8_t PLIERS_BLOCK_LEFT_ID    = 24;
constexpr uint8_t PLIERS_BLOCK_LEFT_IDLE_ANGLE = 224;
constexpr uint8_t PLIERS_BLOCK_LEFT_ACTIVE_ANGLE = 148;

constexpr uint8_t PLIERS_BLOCK_RIGHT_ID   = 18;
constexpr uint8_t PLIERS_BLOCK_RIGHT_IDLE_ANGLE = 76;
constexpr uint8_t PLIERS_BLOCK_RIGHT_ACTIVE_ANGLE = 152;

constexpr uint8_t SLIDER_ELEVATOR_ID = 1;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI;

