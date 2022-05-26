#pragma once
#include "Pliers.hpp"
#include "ServoConfig.hpp"


class PwmPliers : public Pliers {
public:
    PwmPliers(uint8_t id, uint8_t channel, float idleAngle, float activeAngle);
    void init() override;
    void activate() override;
    void deactivate() override;
    void setAngle(float angle) override;
    inline void setConfig(ServoConfig config) override {(void)config; m_shouldUpdateConfig = false;};
    void update() override;
    inline void updateConfig() override {};

protected:
    uint8_t m_channel;

};
