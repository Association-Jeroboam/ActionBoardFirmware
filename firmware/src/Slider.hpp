#pragma once
#include <inttypes.h>
#include <Dynamixel2Arduino.h>
#include "Servo.hpp"
#include "ServoConfig.hpp"


class Slider: public Servo{
public:
    Slider(uint8_t id);
    void init();
    void goToPosition(int16_t position);
    void setAngle(float angle);
    void setConfig(ServoConfig config);
    void setPIDGains(uint16_t p, uint16_t i, uint16_t d);
    void update();
    void updateConfig();
    //TODO: Calibration Method!
    //TODO: Handle rotation direction

private:

    SliderConfig m_config;
    int16_t m_position;

};
