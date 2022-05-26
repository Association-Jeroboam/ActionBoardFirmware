#pragma once
#include "Pliers.hpp"
#include "Dynamixel2Arduino.h"
#include "LocalMath.hpp"
#include "ServoConfig.hpp"

constexpr float    DXL_PLIERS_TORQUE_LIMIT_RAW = 1023.;
constexpr uint16_t DXL_PLIERS_P_GAIN = 32;
constexpr uint16_t DXL_PLIERS_I_GAIN = 10;
constexpr uint16_t DXL_PLIERS_D_GAIN = 0;
constexpr float    DXL_PLIERS_MOVING_SPEED_RAW = 512.;
constexpr float    DXL_PLIERS_MAX_ANGLE_RAD = 300. / 360. * 2 * M_PI; // max angle is 300 degrees

struct DxlPliersStatus {
    float angle;
    float speed;
    float torque;
    float load;
    float temperature;
};

class DxlPliers : public Pliers {
public:
    explicit DxlPliers(uint8_t id, float idleAngle = 90, float activeAngle = 0);
    void init() override;
    void activate() override;
    void deactivate() override;
    void setAngle(float angle) override;
    float getAngle();
    void setConfig(ServoConfig config);
    void reset();
    struct DxlPliersStatus getSatus();
    void update();
    void updateConfig();

private:


    /* These variables are not the current state of the servo
     * They represent the wanted state
     */
    DxlPliersConfig m_config;
};
