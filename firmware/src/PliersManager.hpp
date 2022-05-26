#pragma once
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "Dynamixel2Arduino.h"
#include "DxlPliers.hpp"


constexpr uint16_t PLIERS_MANAGER_WA = 0x100;
constexpr uint8_t  PLIERS_MANAGER_MAX_PLIERS_COUNT = 16;
constexpr uint32_t PLIERS_MANAGER_CAN_UPDATE_PERIOD_MS = 100;
constexpr uint32_t PLIERS_MANAGER_CAN_UPDATE_MSG_PERIOD_US = 500;

enum PlersManagerEventFlags {
    ServoUpdated  = 1 << 0,
    SliderUpdated = 1 << 1,
    SendStates    = 1 << 2,
};

class PliersManager : public chibios_rt::BaseStaticThread<PLIERS_MANAGER_WA>,
                      public CanListener {
public:
    static PliersManager * instance();

    void processCanMsg(CanardRxTransfer * transfer);
    void sendStates();
private:
    PliersManager();
    chibios_rt::EventSource m_eventSource;
    chibios_rt::EventListener m_selflistener;
    void main() override;
    void checkServoUpdate();
    bool doSendStates();
    void subscribeCanTopics();
    void processServoAngle(CanardRxTransfer* transfer);
    void processServoConfig(CanardRxTransfer* transfer);
    void processServoColor(CanardRxTransfer* transfer);
    void processSliderPosition(CanardRxTransfer* transfer);
    void processSliderConfig(CanardRxTransfer* transfer);
    void processPliersStatus(CanardRxTransfer* transfer);
    bool servoProtocolIDToServoID(servoID* servoID, CanProtocolServoID protocolID);
    bool servoIDToservoProtocolID(CanProtocolServoID* protocolID, servoID servoID);
    Servo * m_servo[PLIERS_MANAGER_MAX_PLIERS_COUNT];
    uint8_t m_servoCount;
    uint8_t m_canupdateCount;

    chibios_rt::Timer m_sendStatesTimer;
    chibios_rt::Timer m_sendIndividualStateTimer;

    static PliersManager s_instance;

};
