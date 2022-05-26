#pragma once
#include "cstdint"
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "CanListener.hpp"

constexpr uint32_t PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS = 500;

class PneumaticsManager:public CanListener {
public:
    PneumaticsManager();
    void init();
    void processCanMsg(CanardRxTransfer * transfer);

private:

    void processPumpStatus(CanardRxTransfer* transfer);
    void processValveStatus(CanardRxTransfer* transfer);
    chibios_rt::Timer m_leftValveTimer;
    chibios_rt::Timer m_rightValveTimer;

};
