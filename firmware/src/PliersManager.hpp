#pragma once
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "Dynamixel2Arduino.h"
#include "DxlPliers.hpp"


constexpr uint16_t PLIERS_MANAGER_WA = 0x300;
constexpr uint8_t  PLIERS_MANAGER_MAX_PLIERS_COUNT = 16;

enum PlersManagerEventFlags {
    ServoUpdated  = 1 << 0,
    SliderUpdated = 1 << 0,

};

class PliersManager : public chibios_rt::BaseStaticThread<PLIERS_MANAGER_WA>,
                      public chibios_rt::EventSource,
                      public CanListener {
public:
    static PliersManager * instance();

    void processCanMsg(CanardRxTransfer * transfer);

private:
    PliersManager();
    chibios_rt::EventListener m_selflistener;
    void main() override;
    void processServoAngle(CanardRxTransfer* transfer);
    void processServoConfig(CanardRxTransfer* transfer);
    void processServoColor(CanardRxTransfer* transfer);
    void processSliderPosition(CanardRxTransfer* transfer);
    void processSliderConfig(CanardRxTransfer* transfer);
    void processPumpStatus(CanardRxTransfer* transfer);
    void processValveStatus(CanardRxTransfer* transfer);
    Servo * m_servo[PLIERS_MANAGER_MAX_PLIERS_COUNT];

    static PliersManager s_instance;

};
