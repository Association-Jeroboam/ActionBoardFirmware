#include "Servo.hpp"
#include "Logging.hpp"

Servo::Servo(uint8_t id):
m_id(id),
m_angle(),
m_shouldUpdate(false),
m_shouldUpdateConfig(false)
{}

