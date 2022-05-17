#pragma once
#include "Pliers.hpp"
#include "Dynamixel2Arduino.h"

constexpr float DXL_PLIERS_TORQUE_LIMIT = 20.;
constexpr uint16_t DXL_PLIERS_P_GAIN = 32;
constexpr uint16_t DXL_PLIERS_I_GAIN = 10;
constexpr uint16_t DXL_PLIERS_D_GAIN = 0;
constexpr float    DXL_PLIERS_MOVING_SPEED_PERCENT = 20.;

struct DxlPliersConfig {
    float    torqueLimit;
    float    movingSpeed;
    uint16_t p;
    uint16_t i;
    uint16_t d;
    enum LedColor color;
};

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
    void setAngle(int16_t angle) override;
    void setConfig(DxlPliersConfig config);
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
