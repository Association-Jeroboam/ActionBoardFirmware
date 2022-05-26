#include "PneumaticsManager.hpp"
#include "PumpStatus_0_1.h"
#include "ValveStatus_0_1.h"
#include "Logging.hpp"
#include "Board.hpp"

using namespace Board;


PneumaticsManager::PneumaticsManager():
CanListener(),
m_leftValveTimer(),
m_rightValveTimer()
{
}

static void leftPumpTimeoutCB(virtual_timer_t* timer, void * p) {
    (void)timer;
    (void)p;
    Board::Actuators::setValveState(Actuators::VALVE_LEFT, false);
}

static void rightPumpTimeoutCB(virtual_timer_t* timer, void * p) {
    (void)timer;
    (void)p;
    Board::Actuators::setValveState(Actuators::VALVE_RIGHT, false);
}

void PneumaticsManager::init(){
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_PUMP_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_VALVE_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
}

void PneumaticsManager::processCanMsg(CanardRxTransfer * transfer) {
    switch (transfer->metadata.port_id) {
        case ACTION_PUMP_SET_STATUS_ID:{
            processPumpStatus(transfer);
            break;
        }
        case ACTION_VALVE_SET_STATUS_ID:{
            processValveStatus(transfer);
            break;
        }
        default:
            Logging::println("[PneumaticsManager] Subsription not handled");
            break;
    }

}

void PneumaticsManager::processPumpStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_deserialize_(&pumpStatus,
                                                                        (uint8_t *)transfer->payload,
                                                                        &transfer->payload_size);

    switch(pumpStatus.status.ID) {
        case CAN_PROTOCOL_PUMP_LEFT_ID:
            Logging::println("[PneumaticsManager] Set pump %u : %u", CAN_PROTOCOL_PUMP_LEFT_ID, pumpStatus.status.enabled);
            Board::Actuators::setPumpState(Actuators::PUMP_LEFT, pumpStatus.status.enabled.value);
            break;
        case CAN_PROTOCOL_PUMP_RIGHT_ID:
            Logging::println("[PneumaticsManager] Set pump %u : %u", CAN_PROTOCOL_PUMP_RIGHT_ID, pumpStatus.status.enabled);
            Board::Actuators::setPumpState(Actuators::PUMP_RIGHT, pumpStatus.status.enabled.value);
            break;
        default:
            Logging::println("Unknown Pump ID");
            break;
    }

}

void PneumaticsManager::processValveStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_deserialize_(&valveStatus,
                                                                         (uint8_t *)transfer->payload,
                                                                         &transfer->payload_size);
    switch(valveStatus.status.ID) {
        case CAN_PROTOCOL_PUMP_LEFT_ID:
            Board::Actuators::setValveState(Actuators::VALVE_LEFT, valveStatus.status.enabled.value);
            m_leftValveTimer.set(TIME_MS2I(PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS), leftPumpTimeoutCB, nullptr);
            break;
        case CAN_PROTOCOL_PUMP_RIGHT_ID:
            Board::Actuators::setValveState(Actuators::VALVE_RIGHT, valveStatus.status.enabled.value);
            m_rightValveTimer.set(TIME_MS2I(PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS), rightPumpTimeoutCB, nullptr);
            break;
        default:
            Logging::println("Unknown Valve ID");
            break;
    }
}
