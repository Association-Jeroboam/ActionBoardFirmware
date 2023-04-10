#pragma once
#include "cstdint"
#include "ch.hpp"
#include "CanProtocol.hpp"
#include "CanListener.hpp"

constexpr uint32_t PNEUMATICS_VALVE_ENABLED_TIMEOUT_MS = 500;
constexpr uint32_t TURBINE_MAX_SPEED_TIMEOUT_MS = 10000;
constexpr uint32_t TURBINE_MAX_ENABLED_TIMEOUT_MS = 60000;

class PeripheralManager:public CanListener {
public:
    PeripheralManager();
    void init();
    void processCanMsg(CanardRxTransfer * transfer);

private:

    void processPumpStatus(CanardRxTransfer* transfer);
    void processValveStatus(CanardRxTransfer* transfer);
    void processTurbineStatus(CanardRxTransfer* transfer);
    void processEmergencyState(CanardRxTransfer* transfer);
    chibios_rt::Timer m_leftValveTimer;
    chibios_rt::Timer m_rightValveTimer;
    chibios_rt::Timer m_turbineTimer;
    bool m_emergencyState;

};
