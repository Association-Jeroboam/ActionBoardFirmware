#pragma once
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "Dynamixel2Arduino.h"
#include "DxlPliers.hpp"

constexpr uint16_t RESISTANCE_MEAS_WA = 0x100;
constexpr uint32_t RESISTANCE_MEAS_UPDATE_MS = 100;

class ResistanceMeasure : public chibios_rt::BaseStaticThread<RESISTANCE_MEAS_WA>
{
public:
    ResistanceMeasure();
private:
    void main() override;

    void publishResistanceMeasure(float resistance);
};
