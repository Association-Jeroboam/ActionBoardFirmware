#include "Board.hpp"
#include "Logging.hpp"
#include "PliersManager.hpp"
#include <new>

using namespace Board;

PliersManager PliersManager::s_instance;

PliersManager* PliersManager::instance() {
    return &s_instance;
}

PliersManager::PliersManager() {
    chFifoObjectInit(&m_orderQueue, ORDER_DATA_SIZE, ORDER_QUEUE_LEN,  m_orderBuffer, m_msgBuffer);
}

void PliersManager::main() {
    setName("Pliers Manager");

    for(uint8_t id = 0; id < PLIERS_MANAGER_MAX_PLIERS_COUNT; id++) {
        m_pliers[id] = Actuators::getPliersByID((enum pliersID)id);
        if(m_pliers[id]) {

            m_pliers[id]->deactivate();
        } else {
            Logging::println("[Pliers Manager] Pliers %d Not found!", id);
        }
    }
    Actuators::disengagePliersBlock();
    Actuators::deactivateArm(Actuators::ARM_RIGHT);
    Actuators::deactivateArm(Actuators::ARM_LEFT);

    Com::CANBus::registerListener(this);

    while (!shouldTerminate()){
        canFrame_t * frame;
        chFifoReceiveObjectTimeout(&m_orderQueue, (void **)&frame, TIME_INFINITE);
        Logging::println("[PliersManager] recved order");
        switch (frame->ID) {
            case CAN_PLIERS_ID:
                if( frame->data.pliersData.plierID < PLIERS_MANAGER_MAX_PLIERS_COUNT) {
                    switch (frame->data.pliersData.state) {
                        case PLIERS_IDLE:
                            m_pliers[frame->data.pliersData.plierID]->deactivate();
                            break;
                        case PLIERS_ACTIVATED:
                            m_pliers[frame->data.pliersData.plierID]->activate();
                            break;
                    }

                } else {
                    Logging::println("[PliersManager] received order for unknown pliers");
                }
                break;
            case CAN_PLIERS_BLOCK_ID:
                if(frame->data.pliersBlockData.state == 0){
                    Actuators::disengagePliersBlock();
                } else {
                    Actuators::engagePliersBlock();
                }
                break;
            case CAN_ARMS_ID: {
                enum Board::Actuators::arm arm;
                if ( frame->data.armData.armID == ARM_LEFT ){
                    arm = Actuators::ARM_LEFT;
                } else if ( frame->data.armData.armID == ARM_RIGHT ){
                    arm = Actuators::ARM_RIGHT;
                }
                if( frame->data.armData.state == PLIERS_IDLE) {
                    Board::Actuators::deactivateArm(arm);
                } else if(frame->data.armData.state == PLIERS_ACTIVATED) {
                    Board::Actuators::activateArm(arm);
                }
                break;
            }
            case CAN_SLIDERS_ID:
                break;
            case CAN_FLAG_ID: {
                uint8_t rise = frame->data.flagData.state;
                Logging::print("[PliersManager] flag %u", rise);
                if ( rise ){
                    Board::Actuators::getFlagPliers()->activate();
                } else {
                    Board::Actuators::getFlagPliers()->deactivate();
                }

                break;
            }
            default:
                Logging::println("[PliersManager] Wrong ID received");
        }



        chFifoReturnObject(&m_orderQueue, frame);
    }
}

void PliersManager::processFrame(canFrame_t frame){

    if(frame.ID == CAN_PLIERS_ID       ||
       frame.ID == CAN_PLIERS_BLOCK_ID ||
       frame.ID == CAN_ARMS_ID         ||
       frame.ID == CAN_SLIDERS_ID        ) {
        canFrame_t *pushed_frame = (canFrame_t *) chFifoTakeObjectTimeout(&m_orderQueue, TIME_IMMEDIATE);
        if ( pushed_frame == NULL) {
            Logging::println("[PliersManager] Queue full!");
        } else {
            *pushed_frame = frame;
            chFifoSendObject(&m_orderQueue, pushed_frame);
        }
    }
}
