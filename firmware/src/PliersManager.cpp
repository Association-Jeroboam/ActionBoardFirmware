#include "ch.hpp"
#include "chvt.h"
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

static void sendStatesTimerCB(virtual_timer_t* timer, void* p) {
    chVTDoSetI(timer, TIME_MS2I(PLIERS_MANAGER_CAN_UPDATE_PERIOD_MS), sendStatesTimerCB, nullptr);
    PliersManager::instance()->sendStates();

}

static void sendIndividualStateTimerCB(virtual_timer_t* timer, void* p) {
    PliersManager::instance()->sendStates();
}

PliersManager* PliersManager::instance() {
    return &s_instance;
}

PliersManager::PliersManager():
BaseStaticThread<PLIERS_MANAGER_WA>(),
CanListener(),
m_eventSource(),
m_selflistener(),
m_servoCount(0),
m_canupdateCount(0),
m_sendStatesTimer(),
m_sendIndividualStateTimer(){
}

void PliersManager::main() {
    setName("Pliers Manager");
    chThdSleepMilliseconds(10);

    for(uint8_t id = 0; id < PLIERS_MANAGER_MAX_PLIERS_COUNT; id++) {
        m_servo[id] = Actuators::getServoByID((enum servoID) id);
        if(!m_servo[id]){
            break;
        }
        m_servoCount++;
    }

    m_sendStatesTimer.set(TIME_MS2I(PLIERS_MANAGER_CAN_UPDATE_PERIOD_MS), sendStatesTimerCB, nullptr);

    m_eventSource.registerMask(&m_selflistener, SelfEvent);

    subscribeCanTopics();

    while (!shouldTerminate()){
        eventmask_t event = waitOneEvent(SelfEvent);
        if(event & SelfEvent) {
            eventflags_t flags = m_selflistener.getAndClearFlags();
            if(flags & ServoUpdated) {
                checkServoUpdate();
            }
            if (flags & SendStates ) {
                if(doSendStates()) {
                    m_sendIndividualStateTimer.set(TIME_US2I(PLIERS_MANAGER_CAN_UPDATE_MSG_PERIOD_US), sendStatesTimerCB, nullptr);
                }
            }
        }
    }
}

void PliersManager::checkServoUpdate() {

    for (uint8_t i = 0; i < m_servoCount; i++) {
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

bool PliersManager::doSendStates() {
    static CanardTransferID transfer_id = 0;

    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;

    size_t buf_size = jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    CanardTransferMetadata metadata = {
            .priority = CanardPriorityNominal,
            .transfer_kind = CanardTransferKindMessage,
            .port_id = ACTION_SERVO_CURRENT_ANGLE_ID,
            .remote_node_id = CANARD_NODE_ID_UNSET,
    };


    servoID servo_id = (servoID)m_canupdateCount;
    CanProtocolServoID protocolID;
    if(servoIDToservoProtocolID(&protocolID, servo_id)) {
        metadata.transfer_id = transfer_id;
        servoAngle.ID = protocolID;
        servoAngle.angle.radian = m_servo[0]->getAngle();

        jeroboam_datatypes_actuators_servo_ServoAngle_0_1_serialize_(&servoAngle, buffer, &buf_size);
        Board::Com::CANBus::send(&metadata, buf_size,  buffer);
        transfer_id++;

    }
    m_canupdateCount++;
    if( m_canupdateCount % m_servoCount == 0) {
        m_canupdateCount = 0;
        return false;
    }
    return true;

}

void PliersManager::sendStates() {
    m_eventSource.broadcastFlags(SendStates);
}

void PliersManager::subscribeCanTopics() {
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
        m_eventSource.broadcastFlags(ServoUpdated);
    }
}

void PliersManager::processServoAngle(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1_deserialize_(&servoAngle,
                                                                   (uint8_t *)transfer->payload,
                                                                   &transfer->payload_size);
    servoID servoID;
    if(servoProtocolIDToServoID(&servoID, (CanProtocolServoID)servoAngle.ID)){
        Servo *servo = Actuators::getServoByID(servoID);
        servo->setAngle(servoAngle.angle.radian);
    }
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

    servoID servoID;
    if(servoProtocolIDToServoID(&servoID, (CanProtocolServoID)servoConfig.ID)){
        Servo* servo = Actuators::getServoByID(servoID);
        servo->setConfig(config);
    }
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
    servoID servoID;
    //TODO add wrappers for sliders ID
    if(servoProtocolIDToServoID(&servoID, (CanProtocolServoID)sliderConfig.ID)){
        Servo* servo = Actuators::getServoByID(servoID);
        servo->setConfig(config);
    }

}

bool PliersManager::servoProtocolIDToServoID(servoID* servoID, CanProtocolServoID protocolID) {
    bool success = true;
    switch (protocolID) {
        case CAN_PROTOCOL_SERVO_ARM_LEFT_A:
            *servoID = SERVO_ARM_LEFT_A; break;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_B:
            *servoID = SERVO_ARM_LEFT_B; break;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_C:
            *servoID = SERVO_ARM_LEFT_C; break;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_D:
            *servoID = SERVO_ARM_LEFT_D; break;
        case CAN_PROTOCOL_SERVO_ARM_LEFT_E:
            *servoID = SERVO_ARM_LEFT_E; break;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_A:
            *servoID = SERVO_ARM_RIGHT_A; break;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_B:
            *servoID = SERVO_ARM_RIGHT_B; break;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_C:
            *servoID = SERVO_ARM_RIGHT_C; break;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_D:
            *servoID = SERVO_ARM_RIGHT_D; break;
        case CAN_PROTOCOL_SERVO_ARM_RIGHT_E:
            *servoID = SERVO_ARM_RIGHT_E; break;
        case CAN_PROTOCOL_SERVO_RAKE_LEFT_TOP:
            *servoID = SERVO_RAKE_LEFT_TOP; break;
        case CAN_PROTOCOL_SERVO_RAKE_LEFT_BOTTOM:
            *servoID = SERVO_RAKE_LEFT_BOTTOM; break;
        case CAN_PROTOCOL_SERVO_RAKE_RIGHT_TOP:
            *servoID = SERVO_RAKE_RIGHT_TOP; break;
        case CAN_PROTOCOL_SERVO_RAKE_RIGHT_BOTTOM:
            *servoID = SERVO_RAKE_RIGHT_BOTTOM; break;
        default:
            Logging::println("[PliersManager] Protocol ID not handled %u", protocolID);
            success = false;
    }
    return success;
}

bool PliersManager::servoIDToservoProtocolID(CanProtocolServoID* protocolID, servoID servoID) {
    bool success = true;
    switch (servoID) {
        case SERVO_ARM_LEFT_A:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_LEFT_A; break;
        case SERVO_ARM_LEFT_B:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_LEFT_B; break;
        case SERVO_ARM_LEFT_C:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_LEFT_C; break;
        case SERVO_ARM_LEFT_D:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_LEFT_D; break;
        case SERVO_ARM_LEFT_E:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_LEFT_E; break;
        case SERVO_ARM_RIGHT_A:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_RIGHT_A; break;
        case SERVO_ARM_RIGHT_B:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_RIGHT_B; break;
        case SERVO_ARM_RIGHT_C:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_RIGHT_C; break;
        case SERVO_ARM_RIGHT_D:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_RIGHT_D; break;
        case SERVO_ARM_RIGHT_E:
            *protocolID = CAN_PROTOCOL_SERVO_ARM_RIGHT_E; break;
        case SERVO_RAKE_LEFT_TOP:
            *protocolID = CAN_PROTOCOL_SERVO_RAKE_LEFT_TOP; break;
        case SERVO_RAKE_LEFT_BOTTOM:
            *protocolID = CAN_PROTOCOL_SERVO_RAKE_LEFT_BOTTOM; break;
        case SERVO_RAKE_RIGHT_TOP:
            *protocolID = CAN_PROTOCOL_SERVO_RAKE_RIGHT_TOP; break;
        case SERVO_RAKE_RIGHT_BOTTOM:
            *protocolID = CAN_PROTOCOL_SERVO_RAKE_RIGHT_BOTTOM; break;
        default:
            success = false;
//            Logging::println("[PliersManager] Servo ID  not handled %u", servoID); break;
    }
    return success;

}
