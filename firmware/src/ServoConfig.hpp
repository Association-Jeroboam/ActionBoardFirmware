#ifndef __SERVOCONFIG_HPP__
#define __SERVOCONFIG_HPP__

#include  <inttypes.h>
#include "Dynamixel2Arduino.h"

struct DxlPliersConfig {
    float    torqueLimit;
    float    movingSpeed;
    uint16_t p;
    uint16_t i;
    uint16_t d;
    enum LedColor color;
};

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

struct ServoConfig {
    union {
        DxlPliersConfig dxlPliers;
        SliderConfig    sliderConfig;
    };
};

#endif /* __SERVOCONFIG_HPP__ */
