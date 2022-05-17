#pragma once
#include <inttypes.h>
#include <Dynamixel2Arduino.h>
#include "Servo.hpp"

struct SliderConfig {
    float    torqueLimit;
    float    movingSpeed;
    uint16_t speed_p;
    uint16_t speed_i;
    uint16_t position_p;
    uint16_t position_i;
    uint16_t position_d;
    float    ffwd1;
    float    ffwd2;
};

class Slider: public Servo{
public:
    Slider(uint8_t id);
    void init();
    void goToPosition(int16_t position);
    void setPIDGains(uint16_t p, uint16_t i, uint16_t d);
    void update();
    void updateConfig();
    //TODO: Calibration Method!
    //TODO: Handle rotation direction

private:

    SliderConfig m_config;
    int16_t m_position;

};
