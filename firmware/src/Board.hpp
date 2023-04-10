#pragma once
#include "CanProtocol.hpp"
#include "CanListener.hpp"
#include "Dynamixel2Arduino.h"
#include "Pliers.hpp"

namespace Board {
    void init();

    namespace IO {
        void init();
        void toggleNucleoLed();
        void toggleLed2();
        void toggleLed3();
        float getResistanceMeasure();
    }

    namespace Com {
        void init();
        namespace CANBus {
            void init();
            bool send(const CanardTransferMetadata* const metadata,
                      const size_t                        payload_size,
                      const void* const                   payload);
            bool registerCanMsg(CanListener *listener,
                                CanardTransferKind transfer_kind,
                                CanardPortID port_id,
                                size_t extent);
        }

        namespace DxlServo {
            void init();
            void lockBus();
            void unlockBus();
            Dynamixel2Arduino * getBus();

        }

        namespace I2CBus {
            void init();
            bool transmit(uint8_t addr, uint8_t *txData, uint8_t txLen, uint8_t *rxData, uint8_t rxLen);
            bool receive(uint8_t addr, uint8_t *rxData, uint8_t rxLen);
        }
    }

    namespace Actuators {
        enum arm {
            ARM_LEFT,
            ARM_RIGHT,
        };
        enum Pump {
            PUMP_LEFT  = 0,
            PUMP_RIGHT = 1,
        };
        enum Valve {
            VALVE_LEFT  = 0,
            VALVE_RIGHT = 1,
        };
        enum TurbineSpeed {
            TURBINE_SPEED_STOPPED,
            TURBINE_SPEED_SLOW,
            TURBINE_SPEED_FAST
        };
        void init();
        Servo* getServoByID(enum servoID ID);
        void engagePliersBlock();
        void disengagePliersBlock();
        void activateArm(enum arm);
        void deactivateArm(enum arm);
        void setPumpState(enum Pump, bool enabled, bool fromISR = false);
        void setValveState(enum Valve, bool opened, bool fromISR = false);
        void setTurbineSpeed(enum TurbineSpeed speed, bool fromISR = false);
        enum TurbineSpeed getTurbineSpeed();

        void elevatorSetHeigth(int16_t height);
        void setPwmServo(uint16_t angle);
        Pliers * getFlagPliers();
    }
}