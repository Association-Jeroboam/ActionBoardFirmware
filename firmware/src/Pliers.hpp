#pragma once
#include "Servo.hpp"
#include "Dynamixel2Arduino.h"


enum pliersState {
    PLIERS_IDLE,
    PLIERS_ACTIVATED,
};

class Pliers : public Servo {
public:
    Pliers(uint8_t id, float idleAngle, float activeAngle);
    virtual void init() = 0;
    virtual void activate() = 0;
    virtual void deactivate() = 0;
    inline void setIdleAngle(float idleAngle) { m_idleAngle = idleAngle; };
    inline void setActiveAngle(float activeAngle) { m_activeAngle = activeAngle; };


protected:

    float m_idleAngle;
    float m_activeAngle;

};
