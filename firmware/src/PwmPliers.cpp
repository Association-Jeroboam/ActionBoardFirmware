#include "PwmPliers.hpp"
#include "Board.hpp"
#include "Logging.hpp"
#include "LocalMath.hpp"

PwmPliers::PwmPliers(uint8_t id, uint8_t channel, float idleAngle, float activeAngle): Pliers(id, idleAngle, activeAngle),
                                                                                       m_channel(channel)
                                                                                       {
    m_shouldUpdateConfig = false;

}

void PwmPliers::init(){
    Logging::println("[PwmPliers] init chan %d", m_channel);
    update();
}

void PwmPliers::activate(){
    m_angle = m_activeAngle;
    m_shouldUpdate = true;
}

void PwmPliers::deactivate(){
    m_angle = m_idleAngle;
    m_shouldUpdate = true;
}

void PwmPliers::setAngle(float angle){
    m_angle = angle;
    m_shouldUpdate = true;

}

void PwmPliers::update(){
    const float radToDeg = 1./(2*M_PI)*360;
    Board::Actuators::setPwmServo((uint16_t)(m_angle * radToDeg));
    m_shouldUpdate = false;
}
