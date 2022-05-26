#include "Pliers.hpp"

Pliers::Pliers(uint8_t id, float idleAngle, float activeAngle) :
Servo(id),
m_idleAngle(idleAngle),
m_activeAngle(activeAngle)
{
    m_angle = idleAngle;
}
