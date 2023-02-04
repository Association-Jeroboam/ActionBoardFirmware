#pragma once
#include <inttypes.h>
#include "ServoConfig.hpp"

enum servoID {
#if defined(RED_ROBOT)
    SERVO_ARM_LEFT_A = 0,
    SERVO_ARM_LEFT_B,
    SERVO_ARM_LEFT_C,
    SERVO_ARM_LEFT_D,
    SERVO_ARM_LEFT_E,
    SERVO_ARM_RIGHT_A,
    SERVO_ARM_RIGHT_B,
    SERVO_ARM_RIGHT_C,
    SERVO_ARM_RIGHT_D,
    SERVO_ARM_RIGHT_E,
    SERVO_RAKE_LEFT_TOP,
    SERVO_RAKE_LEFT_BOTTOM,
    SERVO_RAKE_RIGHT_TOP,
    SERVO_RAKE_RIGHT_BOTTOM,
    LEFT_SLIDER,
    RIGHT_SLIDER,
#elif defined(BLUE_ROBOT)
    SERVO_PUSH_ARM_LEFT,
    SERVO_PUSH_ARM_RIGHT,
    SERVO_MEASURE_FORK,
    SERVO_PLIERS_INCLINATION,
    SERVO_PLIERS,
#endif
    SERVO_LAST
};

class Servo {
public:
    Servo(uint8_t id);

    virtual void setAngle(float angle) = 0;
    virtual void setConfig(ServoConfig config) = 0;
    virtual inline float getAngle() = 0;
    virtual void update() = 0;
    virtual void updateConfig() = 0;
    inline bool shouldUpdate() {return m_shouldUpdate;};
    inline bool shouldUpdateConfig() {return m_shouldUpdateConfig;};
    inline void setUpdate() {m_shouldUpdate = true;};
    inline void setUpdateConfig() {m_shouldUpdateConfig = true;};
    inline uint8_t getID() {return m_id;};

protected:
    uint8_t m_id;
    float m_angle;
    bool  m_shouldUpdate;
    bool  m_shouldUpdateConfig;

};
