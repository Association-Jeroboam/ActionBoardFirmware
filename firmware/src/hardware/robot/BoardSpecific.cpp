#include "Board.hpp"
#include "BuildConf.hpp"
#include "PCA9635.h"

static PCA9635 driver(0x40, &I2C_SERVO_DRIVER);

void Board::Actuators::init() {
    driver.begin();
}

void Board::Actuators::setPwmServo(uint8_t channel, uint16_t value) {

}