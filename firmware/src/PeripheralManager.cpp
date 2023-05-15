#include "PeripheralManager.hpp"
#include "PumpStatus_0_1.h"
#include "ValveStatus_0_1.h"
#include "TurbineCmd_0_1.h"
#include "EmergencyState_0_1.h"
#include "Logging.hpp"
#include "Board.hpp"

using namespace Board;


PeripheralManager::PeripheralManager():
CanListener(),
m_leftValveTimer(),
m_rightValveTimer(),
m_turbineTimer(),
m_emergencyState(false)
{
}

using namespace Board::Actuators;

static void leftPumpTimeoutCB(virtual_timer_t* timer, void * p) {
    (void)timer;
    (void)p;
    setValveState(Actuators::VALVE_LEFT, false, true);
}

static void rightPumpTimeoutCB(virtual_timer_t* timer, void * p) {
    (void)timer;
    (void)p;
    setValveState(Actuators::VALVE_RIGHT, false, true);
}

static void turbineTimeoutCB(virtual_timer_t* timer, void * p) {
    (void)timer;
    (void)p;
    TurbineSpeed currentSpeed = getTurbineSpeed();
    if(currentSpeed == TURBINE_SPEED_FAST) {
        // TODO: Fix this
        setTurbineSpeed(TURBINE_SPEED_SLOW, true);
        chSysLockFromISR();
        chVTSetI(timer, TIME_MS2I(TURBINE_MAX_ENABLED_TIMEOUT_MS), turbineTimeoutCB, nullptr);
        chSysUnlockFromISR();
    } else {
        setTurbineSpeed(TURBINE_SPEED_STOPPED, true);
    }
}

void PeripheralManager::init(){
    bool res;
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_PUMP_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_VALVE_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_TURBINE_CMD_ID,
                                jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                EMERGENCY_STATE_ID,
                                jeroboam_datatypes_actuators_common_EmergencyState_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
}

void PeripheralManager::processCanMsg(CanardRxTransfer * transfer) {
    switch (transfer->metadata.port_id) {
        case ACTION_PUMP_SET_STATUS_ID:{
            processPumpStatus(transfer);
            break;
        }
        case ACTION_VALVE_SET_STATUS_ID:{
            processValveStatus(transfer);
            break;
        }
        case ACTION_TURBINE_CMD_ID:{
            processTurbineStatus(transfer);
            break;
        }
        case EMERGENCY_STATE_ID:{
            processEmergencyState(transfer);
            break;
        }
        default:
            Logging::println("[PeripheralManager] Subsription not handled");
            break;
    }

}

void PeripheralManager::processPumpStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_deserialize_(&pumpStatus,
                                                                        (uint8_t *)transfer->payload,
                                                                        &transfer->payload_size);

    switch(pumpStatus.status.ID) {
        case CAN_PROTOCOL_PUMP_LEFT_ID:
            Logging::println("[PeripheralManager] Set pump %u : %u", CAN_PROTOCOL_PUMP_LEFT_ID, pumpStatus.status.enabled);
            setPumpState(Actuators::PUMP_LEFT, pumpStatus.status.enabled.value);
            break;
        case CAN_PROTOCOL_PUMP_RIGHT_ID:
            Logging::println("[PeripheralManager] Set pump %u : %u", CAN_PROTOCOL_PUMP_RIGHT_ID, pumpStatus.status.enabled);
            setPumpState(Actuators::PUMP_RIGHT, pumpStatus.status.enabled.value);
            break;
        default:
            Logging::println("Unknown Pump ID");
            break;
    }

}

void PeripheralManager::processValveStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_deserialize_(&valveStatus,
                                                                         (uint8_t *)transfer->payload,
                                                                         &transfer->payload_size);
    switch(valveStatus.status.ID) {
        case CAN_PROTOCOL_PUMP_LEFT_ID:
            setValveState(Actuators::VALVE_LEFT, valveStatus.status.enabled.value);
            m_leftValveTimer.set(TIME_MS2I(PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS), leftPumpTimeoutCB, nullptr);
            break;
        case CAN_PROTOCOL_PUMP_RIGHT_ID:
            setValveState(Actuators::VALVE_RIGHT, valveStatus.status.enabled.value);
            m_rightValveTimer.set(TIME_MS2I(PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS), rightPumpTimeoutCB, nullptr);
            break;
        default:
            Logging::println("Unknown Valve ID");
            break;
    }
}

void PeripheralManager::processTurbineStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1 command;
    jeroboam_datatypes_actuators_pneumatics_TurbineCmd_0_1_deserialize_(&command,
                                                                        (uint8_t *)transfer->payload,
                                                                        &transfer->payload_size);
    chSysLock();
    bool timerIsArmed = m_turbineTimer.isArmedI();
    chSysUnlock();
    Logging::print("[TURBINE] ");
    switch(command.speed) {
        case CAN_PROTOCOL_TURBINE_SPEED_STOPPED:
            setTurbineSpeed(TURBINE_SPEED_STOPPED);
            Logging::println("Stop");
            break;
        case CAN_PROTOCOL_TURBINE_SPEED_SLOW:
            if(m_emergencyState) break;
            Logging::println("Slow");
            setTurbineSpeed(TURBINE_SPEED_SLOW);
            m_turbineTimer.set(TIME_MS2I(TURBINE_MAX_ENABLED_TIMEOUT_MS), turbineTimeoutCB, nullptr);
            break;
        case CAN_PROTOCOL_TURBINE_SPEED_FAST:
            if(m_emergencyState) break;
            Logging::println("Fast");
            setTurbineSpeed(TURBINE_SPEED_FAST);
            m_turbineTimer.set(TIME_MS2I(TURBINE_MAX_SPEED_TIMEOUT_MS), turbineTimeoutCB, nullptr);
            break;
        default:
            Logging::println("Unknown Speed!");
            break;
    }
}

void PeripheralManager::processEmergencyState(CanardRxTransfer* transfer) {
    jeroboam_datatypes_actuators_common_EmergencyState_0_1 emergencyState;
    jeroboam_datatypes_actuators_common_EmergencyState_0_1_deserialize_(&emergencyState,
                                                                        (uint8_t *)transfer->payload,
                                                                        &transfer->payload_size);
    
    if(m_emergencyState != emergencyState.emergency.value) {
        m_emergencyState = emergencyState.emergency.value;
        if(m_emergencyState) {
            setTurbineSpeed(TURBINE_SPEED_STOPPED);
            setPumpState(Actuators::PUMP_LEFT, false);
            setPumpState(Actuators::PUMP_RIGHT, false);
            setValveState(Actuators::VALVE_LEFT, false);
            setValveState(Actuators::VALVE_RIGHT, false);
            m_leftValveTimer.reset();
            m_rightValveTimer.reset();
            m_turbineTimer.reset    ();
        }
    }
}