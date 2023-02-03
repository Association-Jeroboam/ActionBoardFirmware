#pragma once
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "Dynamixel2Arduino.h"
#include "DxlPliers.hpp"


constexpr uint16_t PLIERS_MANAGER_WA = 0x200;
constexpr uint8_t  PLIERS_MANAGER_MAX_PLIERS_COUNT = 16;
constexpr uint32_t PLIERS_MANAGER_CAN_UPDATE_PERIOD_MS = 100;
constexpr uint32_t PLIERS_MANAGER_CAN_UPDATE_MSG_PERIOD_US = 500;

constexpr uint16_t MSG_QUEUE_LEN  = 10;
constexpr uint16_t MSG_DATA_SIZE = sizeof(CanardRxTransfer);

enum PlersManagerEventFlags {
    ServoUpdated     = 1 << 0,
    SliderUpdated    = 1 << 1,
    SendStates       = 1 << 2,
    EmergencyCleared = 1 << 3,
    OrderReceived    = 1 << 4,
};

class PliersManager : public chibios_rt::BaseStaticThread<PLIERS_MANAGER_WA>,
                      public CanListener {
public:
    static PliersManager * instance();

    void processCanMsg(CanardRxTransfer * transfer);
    void sendStates();
    void updateServos();
private:
    PliersManager();
    chibios_rt::EventSource m_eventSource;
    chibios_rt::EventListener m_selflistener;
    void main() override;
    void checkServoUpdate();
    bool doSendStates();
    void subscribeCanTopics();
    void applyOrder(CanardRxTransfer * transfer);
    void processServoAngle(CanardRxTransfer* transfer);
    void processServoConfig(CanardRxTransfer* transfer);
    void processServoColor(CanardRxTransfer* transfer);
    void processSliderPosition(CanardRxTransfer* transfer);
    void processSliderConfig(CanardRxTransfer* transfer);
    void processPliersStatus(CanardRxTransfer* transfer);
    void processEmergencyState(CanardRxTransfer* transfer);
	void processServoReboot(CanardRxTransfer* transfer);
    void servoForceUpdate();
    bool servoProtocolIDToServoID(servoID* servoID, CanProtocolServoID protocolID);
    bool servoIDToservoProtocolID(CanProtocolServoID* protocolID, servoID servoID);

	objects_fifo_t pendingMessagesQueue;
	CanardRxTransfer    dataBuffer[MSG_QUEUE_LEN];
	msg_t          msgBuffer[MSG_QUEUE_LEN];

    Servo * m_servo[PLIERS_MANAGER_MAX_PLIERS_COUNT];
    uint8_t m_servoCount;
    uint8_t m_canupdateCount;
    bool m_lastEmgcyState;

    chibios_rt::Timer m_sendStatesTimer;
    chibios_rt::Timer m_sendIndividualStateTimer;

    static PliersManager s_instance;

};
