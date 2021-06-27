#include "Board.hpp"
#include "BuildConf.hpp"
#include "Adafruit_PWMServoDriver.h"
#include "Logging.hpp"

static Adafruit_PWMServoDriver driver(0x40, &I2C_SERVO_DRIVER);

void Board::Actuators::init() {
    Logging::println("[BoardSpecific] init Actuators");
    driver.begin();
    driver.setOscillatorFrequency(27050000);
    driver.setPWMFreq(50);
}

void Board::Actuators::setPwmServo(uint8_t channel, uint16_t value) {
    driver.setPWM(channel, 0, value);
}