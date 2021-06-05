#pragma once
#include "Pliers.hpp"
#include "Dynamixel2Arduino.h"

constexpr float DXL_PLIERS_TORQUE_LIMIT = 20.;
constexpr uint16_t DXL_PLIERS_P_GAIN = 32;
constexpr uint16_t DXL_PLIERS_I_GAIN = 0;
constexpr uint16_t DXL_PLIERS_D_GAIN = 0;

class DxlPliers : public Pliers {
public:
    DxlPliers(uint8_t id, float idleAngle = 90, float activeAngle = 0);
    void init();
    void open();
    void close();

};
