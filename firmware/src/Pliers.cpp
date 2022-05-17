#include "Pliers.hpp"

Pliers::Pliers(uint8_t id, float idleAngle, float activeAngle) :
Servo(id),
m_angle(idleAngle),
m_idleAngle(idleAngle),
m_activeAngle(activeAngle)
{}
