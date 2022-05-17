#pragma once
#include <inttypes.h>

class Servo {
public:
    Servo(uint8_t id);

    virtual void update() = 0;
    virtual void updateConfig() = 0;
    inline bool shouldUpdate() {return m_shouldUpdate;};
    inline bool shouldUpdateConfig() {return m_shouldUpdateConfig;};

protected:
    uint8_t m_id;
    bool  m_shouldUpdate;
    bool  m_shouldUpdateConfig;

};
