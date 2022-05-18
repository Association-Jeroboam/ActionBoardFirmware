#include "ch.hpp"
#include "Board.hpp"
#include "Logging.hpp"
#include "PliersManager.hpp"
#include <new>
#include "ServoAngle_0_1.h"
#include "ServoConfig_0_1.h"
#include "SliderPosition_0_1.h"
#include "SliderConfig_0_1.h"
#include "ServoConfig.hpp"

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

    while (!shouldTerminate()){
        eventmask_t event = waitOneEvent(SelfEvent);
        if(event & SelfEvent) {
            eventflags_t flags = m_selflistener.getAndClearFlags();
            if(flags & ServoUpdated) {
                for (uint8_t i = 0; i < PLIERS_MANAGER_MAX_PLIERS_COUNT; i++) {
                    if(m_servo[i]) {
                        if (m_servo[i]->shouldUpdate() ){
                            Logging::println("Update servo %u", i);
                            m_servo[i]->update();
                        }
                        if (m_servo[i]->shouldUpdateConfig() ){
                            Logging::println("Update servo config %u", i);
                            m_servo[i]->updateConfig();
                        }
                    }
                }
            }
        }
    }
}

void PliersManager::processCanMsg(CanardRxTransfer * transfer){
    bool broadcastFlags = true;
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
            Logging::println("[PliersManager] process ACTION_SERVO_SET_COLOR");
            processServoColor(transfer);
            break;
        }
        case ACTION_SLIDER_SET_POSITION_ID:{
            Logging::println("[PliersManager] process ACTION_SLIDER_SET_POSITION");
            processSliderPosition(transfer);
            break;
        }
        case ACTION_SLIDER_SET_CONFIG_ID:{
            Logging::println("[PliersManager] process ACTION_SLIDER_SET_CONFIG");
            processSliderConfig(transfer);
            break;
        }
        default:
            broadcastFlags = false;
            Logging::println("[PliersManager] Subsription not handled");
            break;
    }
    if(broadcastFlags) {
        this->broadcastFlags(ServoUpdated);
    }
}

void PliersManager::processServoAngle(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1_deserialize_(&servoAngle,
                                                                   (uint8_t *)transfer->payload,
                                                                   &transfer->payload_size);
    Servo* servo = Actuators::getServoByID(servoProtocolIDToServoID((CanProtocolServoID)servoAngle.ID));
    servo->setAngle(servoAngle.angle.radian);
}

void PliersManager::processServoConfig(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoConfig_0_1 servoConfig;
    jeroboam_datatypes_actuators_servo_ServoConfig_0_1_deserialize_(&servoConfig,
                                                                    (uint8_t *)transfer->payload,
                                                                    &transfer->payload_size);
    ServoConfig config;
    config.dxlPliers.color = LedColor::LED_GREEN;
    config.dxlPliers.torqueLimit = servoConfig._torque_limit;
    config.dxlPliers.movingSpeed = servoConfig.moving_speed;
    config.dxlPliers.p = servoConfig.pid.pid[0];
    config.dxlPliers.i = servoConfig.pid.pid[1];
    config.dxlPliers.d = servoConfig.pid.pid[2];
    (void)servoConfig.pid.bias; //discard it for now

    Servo* servo = Actuators::getServoByID(servoProtocolIDToServoID((CanProtocolServoID)servoConfig.ID));
    servo->setConfig(config);
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
    ServoConfig config;
    config.sliderConfig.torqueLimit = 100;
    config.sliderConfig.movingSpeed = 100;
    config.sliderConfig.position_p = sliderConfig.position_pid.pid[0];
    config.sliderConfig.position_i = sliderConfig.position_pid.pid[1];
    config.sliderConfig.position_d = sliderConfig.position_pid.pid[2];
    config.sliderConfig.speed_p = sliderConfig.speed_pi.pid[0];
    config.sliderConfig.speed_i = sliderConfig.speed_pi.pid[1];
    config.sliderConfig.ffwd1 = sliderConfig.feedforward_gains[0];
    config.sliderConfig.ffwd2 = sliderConfig.feedforward_gains[1];

    Servo* servo = Actuators::getServoByID(servoProtocolIDToServoID((CanProtocolServoID)sliderConfig.ID));
    servo->setConfig(config);
}

servoID PliersManager::servoProtocolIDToServoID(CanProtocolServoID protocolID) {
    switch (protocolID) {
        case CAN_PROTOCOL_SERVO_ARM_LEFT_A:
            return SERVO_ARM_LEFT_A;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_B:
            return SERVO_ARM_LEFT_B;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_C:
            return SERVO_ARM_LEFT_C;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_D:
            return SERVO_ARM_LEFT_D;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_E:
            return SERVO_ARM_LEFT_E;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_A:
            return SERVO_ARM_RIGHT_A;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_B:
            return SERVO_ARM_RIGHT_B;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_C:
            return SERVO_ARM_RIGHT_C;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_D:
            return SERVO_ARM_RIGHT_D;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_E:
            return SERVO_ARM_RIGHT_E;
    }
}
