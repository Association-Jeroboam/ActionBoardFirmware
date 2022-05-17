#include "ch.hpp"
#include "Board.hpp"
#include "Logging.hpp"
#include "PliersManager.hpp"
#include <new>
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"
#include "SliderPosition_0_1.h"
#include "SliderConfig_0_1.h"
#include "PumpStatus_0_1.h"
#include "ValveStatus_0_1.h"

enum PliersManagerEvents {
    SelfEvent      = 1 << 0,
};

using namespace Board;

PliersManager PliersManager::s_instance;

PliersManager* PliersManager::instance() {
    return &s_instance;
}

PliersManager::PliersManager():
BaseStaticThread<PLIERS_MANAGER_WA>(),
chibios_rt::EventSource(),
CanListener(){
}

void PliersManager::main() {
    setName("Pliers Manager");

    for(uint8_t id = 0; id < PLIERS_MANAGER_MAX_PLIERS_COUNT; id++) {
        m_servo[id] = Actuators::getServoByID((enum servoID) id);
    }

    this->registerMask(&m_selflistener, SelfEvent);

    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_SET_ANGLE_ID,
                                jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_SET_CONFIG_ID,
                                jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    //TODO
//    Com::CANBus::registerCanMsg(this,
//                                CanardTransferKindMessage,
//                                ACTION_SERVO_SET_COLOR_ID,
//                                );
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SLIDER_SET_POSITION_ID,
                                jeroboam_datatypes_actuators_servo_SliderPosition_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SLIDER_SET_CONFIG_ID,
                                jeroboam_datatypes_actuators_servo_SliderConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_PUMP_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_VALVE_SET_STATUS_ID,
                                jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);

    while (!shouldTerminate()){
        eventmask_t event = waitOneEvent(SelfEvent);

        if(event & SelfEvent) {
            eventflags_t flags = m_selflistener.getAndClearFlags();
            if(flags & ServoUpdated) {
                for (uint8_t i = 0; i < PLIERS_MANAGER_MAX_PLIERS_COUNT; i++) {
                    if(m_servo[i]) {
                        if (m_servo[i]->shouldUpdate() ){
                            m_servo[i]->update();
                        }
                        if (m_servo[i]->shouldUpdateConfig() ){
                            m_servo[i]->updateConfig();
                        }
                    }
                }
            }
        }
    }
}

void PliersManager::processCanMsg(CanardRxTransfer * transfer){
    switch (transfer->metadata.port_id) {
        case ACTION_SERVO_SET_ANGLE_ID:{
            processServoAngle(transfer);
            break;
        }
        case ACTION_SERVO_SET_CONFIG_ID:{
            processServoConfig(transfer);
            break;
        }
        case ACTION_SERVO_SET_COLOR_ID:{
            processServoColor(transfer);
            break;
        }
        case ACTION_SLIDER_SET_POSITION_ID:{
            processSliderPosition(transfer);
            break;
        }
        case ACTION_SLIDER_SET_CONFIG_ID:{
            processSliderConfig(transfer);
            break;
        }
        case ACTION_PUMP_SET_STATUS_ID:{
            processPumpStatus(transfer);
            break;
        }
        case ACTION_VALVE_SET_STATUS_ID:{
            processValveStatus(transfer);
            break;
        }
        default:
            Logging::println("[PliersManager] Subsription not handled");
            break;
    }
}

void PliersManager::processServoAngle(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1_deserialize_(&servoAngle,
                                                                   (uint8_t *)transfer->payload,
                                                                   &transfer->payload_size);

}

void PliersManager::processServoConfig(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoConfig_0_1 servoConfig;
    jeroboam_datatypes_actuators_servo_ServoConfig_0_1_deserialize_(&servoConfig,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);

}

void PliersManager::processServoColor(CanardRxTransfer* transfer){
    //TODO
}

void PliersManager::processSliderPosition(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_SliderPosition_0_1 sliderPosition;
    jeroboam_datatypes_actuators_servo_SliderPosition_0_1_deserialize_(&sliderPosition,
                                                                       (uint8_t *)transfer->payload,
                                                                       &transfer->payload_size);

}

void PliersManager::processSliderConfig(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_SliderConfig_0_1 sliderConfig;
    jeroboam_datatypes_actuators_servo_SliderConfig_0_1_deserialize_(&sliderConfig,
                                                                     (uint8_t *)transfer->payload,
                                                                     &transfer->payload_size);

}

void PliersManager::processPumpStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1 pumpStatus;
    jeroboam_datatypes_actuators_pneumatics_PumpStatus_0_1_deserialize_(&pumpStatus,
                                                                        (uint8_t *)transfer->payload,
                                                                        &transfer->payload_size);

}

void PliersManager::processValveStatus(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1 valveStatus;
    jeroboam_datatypes_actuators_pneumatics_ValveStatus_0_1_deserialize_(&valveStatus,
                                                                         (uint8_t *)transfer->payload,
                                                                         &transfer->payload_size);

}
