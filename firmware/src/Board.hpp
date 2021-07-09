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
    }

    namespace Com {
        void init();
        namespace CANBus {
            void init();
            bool send(canFrame_t canData);
            void registerListener(CanListener * listener);
        }

        namespace DxlServo {
            void init();
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
        void init();
        Pliers* getPliersByID(enum pliersID ID);
        void engagePliersBlock();
        void disengagePliersBlock();
        void activateArm(enum arm);
        void deactivateArm(enum arm);
        void elevatorSetHeigth(int16_t height);
        void setPwmServo(uint8_t channel, uint16_t value);
        Pliers * getFlagPliers();
    }
}