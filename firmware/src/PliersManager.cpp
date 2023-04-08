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
#include "PliersStatus_0_1.h"
#include "ServoConfig.hpp"
#include "EmergencyState_0_1.h"
#include "ServoID_0_1.h"
#include "GenericCommand_0_1.h"
#include "GenericRead_0_1.h"
#include "GenericReadResponse_0_1.h"

enum PliersManagerEvents {
    SelfEvent      = 1 << 0,
};

using namespace Board;

PliersManager PliersManager::s_instance;

static void sendStatesTimerCB(virtual_timer_t* timer, void* p) {
    (void)p;
    chSysLockFromISR();
    chVTSetI(timer, TIME_MS2I(PLIERS_MANAGER_CAN_UPDATE_PERIOD_MS), sendStatesTimerCB, nullptr);
    chSysUnlockFromISR();
    PliersManager::instance()->sendStates();

}

static void sendIndividualStateTimerCB(virtual_timer_t* timer, void* p) {
    (void)timer;
    (void)p;
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
m_lastEmgcyState(true),
m_sendStatesTimer(),
m_sendIndividualStateTimer(){
	chFifoObjectInit(&pendingMessagesQueue, MSG_DATA_SIZE, MSG_QUEUE_LEN,  dataBuffer, msgBuffer);
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

    servoForceUpdate();

    while (!shouldTerminate()){
        eventmask_t event = waitOneEvent(SelfEvent);
        if(event & SelfEvent) {
            eventflags_t flags = m_selflistener.getAndClearFlags();
            if(flags & ServoUpdated && !m_lastEmgcyState) {
                checkServoUpdate();
            }
            if (flags & SendStates && !m_lastEmgcyState) {
                if(doSendStates()) {
                    m_sendIndividualStateTimer.set(TIME_US2I(PLIERS_MANAGER_CAN_UPDATE_MSG_PERIOD_US), sendIndividualStateTimerCB, nullptr);
                }
            }

            if(flags & EmergencyCleared) {
                chThdSleepMilliseconds(800);
                servoForceUpdate();
            }

            if(flags & OrderReceived) {
                CanardRxTransfer * transfer;
                while(chFifoReceiveObjectTimeout(&pendingMessagesQueue, (void **)&transfer, TIME_IMMEDIATE) == MSG_OK) {
                    CanardRxTransfer transfer_copy = *transfer;
                    chFifoReturnObject(&pendingMessagesQueue, transfer);
                    applyOrder(&transfer_copy);
                }
            }
        }

    }
}

void PliersManager::servoForceUpdate() {
    for (uint8_t i = 0; i < m_servoCount; i++) {
        if(m_servo[i]) {
            m_servo[i]->updateConfig();
            m_servo[i]->update();
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
            .transfer_id = 0,
    };


    servoID servo_id = (servoID)m_canupdateCount;
    CanProtocolServoID protocolID;
    if(servoIDToservoProtocolID(&protocolID, servo_id)) {
        metadata.transfer_id = transfer_id;
        servoAngle.ID = protocolID;
        servoAngle.angle.radian = m_servo[m_canupdateCount]->getAngle();

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
    chSysLockFromISR();
    m_eventSource.broadcastFlagsI(SendStates);
    chSysUnlockFromISR();
}

void PliersManager::subscribeCanTopics() {
    bool res;

    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_SET_ANGLE_ID,
                                jeroboam_datatypes_actuators_servo_ServoAngle_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_SET_ANGLE_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_SET_CONFIG_ID,
                                jeroboam_datatypes_actuators_servo_ServoConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_SET_CONFIG_ID");}
    //TODO
//    Com::CANBus::registerCanMsg(this,
//                                CanardTransferKindMessage,
//                                ACTION_SERVO_SET_COLOR_ID,
//                                );
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SLIDER_SET_POSITION_ID,
                                jeroboam_datatypes_actuators_servo_SliderPosition_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SLIDER_SET_POSITION_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SLIDER_SET_CONFIG_ID,
                                jeroboam_datatypes_actuators_servo_SliderConfig_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SLIDER_SET_CONFIG_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_SET_PLIERS_ID,
                                jeroboam_datatypes_actuators_servo_PliersStatus_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_SET_PLIERS_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                EMERGENCY_STATE_ID,
                                jeroboam_datatypes_actuators_common_EmergencyState_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ,");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_REBOOT_ID,
                                jeroboam_datatypes_actuators_servo_ServoID_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_REBOOT_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindMessage,
                                ACTION_SERVO_GENERIC_COMMAND_ID,
                                jeroboam_datatypes_actuators_servo_GenericCommand_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_GENERIC_COMMAND_ID");}
    res = Com::CANBus::registerCanMsg(this,
                                CanardTransferKindRequest,
                                ACTION_SERVO_GENERIC_READ_ID,
                                jeroboam_datatypes_actuators_servo_GenericRead_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_);
    if(!res) { Logging::println("Error subscribing ACTION_SERVO_GENERIC_READ_ID");}
}

void PliersManager::processCanMsg(CanardRxTransfer * transfer){
    switch (transfer->metadata.port_id) {
        case ACTION_SERVO_SET_ANGLE_ID:
        case ACTION_SERVO_SET_CONFIG_ID:
        case ACTION_SERVO_SET_COLOR_ID:
        case ACTION_SLIDER_SET_POSITION_ID:
        case ACTION_SLIDER_SET_CONFIG_ID:
        case ACTION_SERVO_SET_PLIERS_ID:
        case ACTION_SERVO_REBOOT_ID:
        case ACTION_SERVO_GENERIC_COMMAND_ID:
        case ACTION_SERVO_GENERIC_READ_ID:{
            Logging::println("process msg");
            CanardRxTransfer *newTransfer = (CanardRxTransfer *) chFifoTakeObjectTimeout(&pendingMessagesQueue, TIME_IMMEDIATE);
            if(newTransfer == NULL) {
                Logging::println("[Pliers Manager] Queue full!");
            } else {
                *newTransfer = *transfer;
                chFifoSendObject(&pendingMessagesQueue, newTransfer);
                m_eventSource.broadcastFlags(OrderReceived);
            }
            break;
        }
        // Hack because Emergency state is broadcasted periodically fast and it kills the thread
        case EMERGENCY_STATE_ID :{
            processEmergencyState(transfer);
            break;
        }
        default:
            Logging::println("[PliersManager] Subsription not handled");
            break;
    }
}

void PliersManager::applyOrder(CanardRxTransfer * transfer) {
    bool broadcastFlags = true;
    switch (transfer->metadata.port_id) {
        case ACTION_SERVO_SET_ANGLE_ID:{
//            Logging::println("[PliersManager] process ACTION_SERVO_SET_ANGLE_ID");
            processServoAngle(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
            break;
        }
        case ACTION_SERVO_SET_CONFIG_ID:{
            processServoConfig(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
//            Logging::println("[PliersManager] process ACTION_SERVO_SET_CONFIG");
            break;
        }
        case ACTION_SERVO_SET_COLOR_ID:{
//            Logging::println("[PliersManager] process ACTION_SERVO_SET_COLOR");
            processServoColor(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
            break;
        }
        case ACTION_SLIDER_SET_POSITION_ID:{
//            Logging::println("[PliersManager] process ACTION_SLIDER_SET_POSITION");
            processSliderPosition(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
            break;
        }
        case ACTION_SLIDER_SET_CONFIG_ID:{
            Logging::println("[PliersManager] process ACTION_SLIDER_SET_CONFIG");
            processSliderConfig(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
            break;
        }
        case ACTION_SERVO_SET_PLIERS_ID:{
            Logging::println("[PliersManager] process ACTION_SERVO_SET_PLIERS");
            processPliersStatus(transfer);
            m_eventSource.broadcastFlags(ServoUpdated);
            break;
        }
        case ACTION_SERVO_REBOOT_ID: {
            Logging::println("[PliersManager] process ACTION_SERVO_REBOOT_ID");
            processServoReboot(transfer);
            break;
        }
        case ACTION_SERVO_GENERIC_COMMAND_ID:{
            Logging::println("[PliersManager] process ACTION_SERVO_GENERIC_COMMAND_ID");
            processServoGenericCommand(transfer);
            break;
        }
        case ACTION_SERVO_GENERIC_READ_ID:{
            Logging::println("[PliersManager] process ACTION_SERVO_GENERIC_READ_ID");
            processServoGenericRead(transfer);
            break;
        }
        default:
            broadcastFlags = false;
            break;
    }
    if(broadcastFlags) {
        updateServos();
    }
}

void PliersManager::updateServos() {
    m_eventSource.broadcastFlags(ServoUpdated);
}

void PliersManager::processServoAngle(CanardRxTransfer* transfer){
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1 servoAngle;
    jeroboam_datatypes_actuators_servo_ServoAngle_0_1_deserialize_(&servoAngle,
                                                                   (uint8_t *)transfer->payload,
                                                                   &transfer->payload_size);
    servoID servoID;
    if(servoProtocolIDToServoID(&servoID, (CanProtocolServoID)servoAngle.ID)){
        Servo *servo = Actuators::getServoByID(servoID);
        if(servo){
            servo->setAngle(servoAngle.angle.radian);
        } else {
            Logging::println("Servo not found!");
        }
    } else {
        Board::Com::DxlServo::lockBus();
        Board::Com::DxlServo::getBus()->setGoalPosition(servoAngle.ID, servoAngle.angle.radian * (360./ (2*M_PI)), UNIT_DEGREE);
        Board::Com::DxlServo::unlockBus();
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
    } else {
        Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
        Board::Com::DxlServo::lockBus();
        bus->torqueOff(servoConfig.ID);
        bus->setPositionPIDGain(servoConfig.ID,
                                servoConfig.pid.pid[0],
                                servoConfig.pid.pid[1],
                                servoConfig.pid.pid[2]);
        bus->setTorqueLimit(servoConfig.ID, servoConfig._torque_limit, UNIT_RAW);
        bus->setGoalVelocity(servoConfig.ID, servoConfig.moving_speed, UNIT_RAW); //TODO check unit conversion
        bus->torqueOn(servoConfig.ID);
        Board::Com::DxlServo::unlockBus();

        Logging::println("PositionPIDGain %u %u %u", servoConfig.pid.pid[0], servoConfig.pid.pid[1], servoConfig.pid.pid[2] );
        Logging::println("TorqueLimit %u", servoConfig._torque_limit );
        Logging::println("GoalVelocity %u", servoConfig.moving_speed );
    }
}

void PliersManager::processServoColor(CanardRxTransfer* transfer){
    (void)transfer;
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

void PliersManager::processPliersStatus(CanardRxTransfer* transfer) {
    jeroboam_datatypes_actuators_servo_PliersStatus_0_1 pliersStatus;
    jeroboam_datatypes_actuators_servo_PliersStatus_0_1_deserialize_(&pliersStatus,
                                                                     (uint8_t *)transfer->payload,
                                                                     &transfer->payload_size);
#if defined(RED_ROBOT)
#elif defined(BLUE_ROBOT)
    Pliers* pliers = (Pliers*)Actuators::getServoByID(SERVO_PLIERS);
    if(pliersStatus.active.value) {
        pliers->activate();
    } else {
        pliers->deactivate();
    }
#endif
}

void PliersManager::processEmergencyState(CanardRxTransfer* transfer) {
    jeroboam_datatypes_actuators_common_EmergencyState_0_1 emgcyState;

    jeroboam_datatypes_actuators_common_EmergencyState_0_1_deserialize_(&emgcyState, (uint8_t*)transfer->payload, &transfer->payload_size);

    if(emgcyState.emergency.value != m_lastEmgcyState) {
        if(emgcyState.emergency.value) {
            Logging::println("mgcy set");
        } else {
            m_eventSource.broadcastFlags(EmergencyCleared);
            Logging::println("mgcy cleared");
        }
        m_lastEmgcyState = emgcyState.emergency.value;

    }

}
void PliersManager::processServoReboot(CanardRxTransfer* transfer) {

    jeroboam_datatypes_actuators_servo_ServoID_0_1 rebootID;
    jeroboam_datatypes_actuators_servo_ServoID_0_1_deserialize_(&rebootID, (uint8_t*)transfer->payload, &transfer->payload_size);
    servoID servoID;
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    if(servoProtocolIDToServoID(&servoID, (CanProtocolServoID)rebootID.ID)) {
        Servo* servo = Actuators::getServoByID(servoID);
        bus->reboot(servo->getID(), 10);
        servo->setUpdateConfig();
        servo->setUpdate();
        chThdSleepMilliseconds(100);
        m_eventSource.broadcastFlags(ServoUpdated);
    } else {
        bus->reboot(rebootID.ID, 10);
        m_eventSource.broadcastFlags(EmergencyCleared);
    }
}

void PliersManager::processServoGenericCommand(CanardRxTransfer* transfer) {
    jeroboam_datatypes_actuators_servo_GenericCommand_0_1 command;
    jeroboam_datatypes_actuators_servo_GenericCommand_0_1_deserialize_(&command, (uint8_t*)transfer->payload, &transfer->payload_size);
    Logging::print("ID: %u addr %u data ", command.id, command.addr);

    for (size_t i = 0; i < command.data.count; ++i) {
        Logging::print("%u ", command.data.elements[i]);
    }
    Logging::println("");
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    bool ret = bus->write(command.id, command.addr, command.data.elements, command.data.count);
    if(!ret) {
        Logging::println("failed!");
    }

}

void PliersManager::processServoGenericRead(CanardRxTransfer* transfer) {
    jeroboam_datatypes_actuators_servo_GenericRead_0_1 read;
    jeroboam_datatypes_actuators_servo_GenericRead_0_1_deserialize_(&read, (uint8_t*)transfer->payload, &transfer->payload_size);
    Logging::print("ID: %u addr %u len %u ", read.id, read.addr, read.len);

    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    const uint16_t buf_len = 2*jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_data_ARRAY_CAPACITY_;
    uint8_t buf[buf_len];
    int32_t ret = bus->read(read.id, read.addr, read.len, buf, buf_len, 30);

    jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1 response;


    if(ret == read.len) {
        Logging::println("success!");
        Logging::println("len %li", ret);
        response._tag_ = CAN_PROTOCOL_RESPONSE_SUCCESS;
        response.data.count = read.len;
        for(uint16_t i = 0; i < read.len; i++) {
            response.data.elements[i] = buf[2*i];
            response.data.elements[i] = buf[2*i+1] << 8;
        }

    } else {
        Logging::println("failed with code %li", bus->getLastLibErrCode());
        Logging::println("len %li", ret);
        response._tag_ = CAN_PROTOCOL_RESPONSE_ERROR;
        response.err_code = ret;
    }

    size_t  response_size = jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t response_buffer[jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    CanardTransferMetadata metadata = {
        .priority = CanardPriorityNominal,
        .transfer_kind = CanardTransferKindResponse,
        .port_id = ACTION_SERVO_GENERIC_READ_ID,
        .remote_node_id = CAN_PROTOCOL_EMBEDDED_COMPUTER_ID,
        .transfer_id = transfer->metadata.transfer_id,
    };
    jeroboam_datatypes_actuators_servo_GenericReadResponse_0_1_serialize_(&response, response_buffer, &response_size);
    Board::Com::CANBus::send(&metadata, response_size,  response_buffer);
}

bool PliersManager::servoProtocolIDToServoID(servoID* servoID, CanProtocolServoID protocolID) {
    bool success = true;
    switch (protocolID) {
#if defined(RED_ROBOT)
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
#elif defined(BLUE_ROBOT)
        case CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT:
            *servoID = SERVO_PUSH_ARM_LEFT; break;
        case CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT:
            *servoID = SERVO_PUSH_ARM_RIGHT; break;
        case CAN_PROTOCOL_SERVO_MEASURE_FORK:
            *servoID = SERVO_MEASURE_FORK; break;
        case CAN_PROTOCOL_SERVO_PLIERS_INCLINATION:
            *servoID = SERVO_PLIERS_INCLINATION; break;
        case CAN_PROTOCOL_SERVO_PLIERS:
            *servoID = SERVO_PLIERS; break;
#endif
        default:
            Logging::println("[PliersManager] Protocol ID not handled %u", protocolID);
            success = false;
    }
    return success;
}

bool PliersManager::servoIDToservoProtocolID(CanProtocolServoID* protocolID, servoID servoID) {
    bool success = true;
    switch (servoID) {
#if defined(RED_ROBOT)
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
#elif defined(BLUE_ROBOT)
        case SERVO_PUSH_ARM_LEFT:
            *protocolID = CAN_PROTOCOL_SERVO_PUSH_ARM_LEFT; break;
        case SERVO_PUSH_ARM_RIGHT:
            *protocolID = CAN_PROTOCOL_SERVO_PUSH_ARM_RIGHT; break;
        case SERVO_MEASURE_FORK:
            *protocolID = CAN_PROTOCOL_SERVO_MEASURE_FORK; break;
        case SERVO_PLIERS_INCLINATION:
            *protocolID = CAN_PROTOCOL_SERVO_PLIERS_INCLINATION; break;
        case SERVO_PLIERS:
            *protocolID = CAN_PROTOCOL_SERVO_PLIERS; break;
#endif
        default:
            success = false;
//            Logging::println("[PliersManager] Servo ID  not handled %u", servoID); break;
    }
    return success;

}
