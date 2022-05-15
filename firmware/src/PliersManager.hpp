#pragma once
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "Dynamixel2Arduino.h"
#include "DxlPliers.hpp"


constexpr uint16_t PLIERS_MANAGER_WA = 0x300;
//constexpr uint16_t ORDER_DATA_SIZE = sizeof(canFrame_t);
//constexpr uint16_t ORDER_QUEUE_LEN = 10;
constexpr uint8_t  PLIERS_MANAGER_MAX_PLIERS_COUNT = 9;

class PliersManager : public chibios_rt::BaseStaticThread<PLIERS_MANAGER_WA>,
                      public CanListener {
public:
    static PliersManager * instance();

    void processCanMsg(CanardRxTransfer * transfer);

private:
    PliersManager();
    void main() override;
//    objects_fifo_t m_orderQueue;
//    canFrame_t     m_orderBuffer[ORDER_QUEUE_LEN];
//    msg_t          m_msgBuffer[ORDER_QUEUE_LEN];
    Pliers *       m_pliers[PLIERS_MANAGER_MAX_PLIERS_COUNT];

    static PliersManager s_instance;

};
