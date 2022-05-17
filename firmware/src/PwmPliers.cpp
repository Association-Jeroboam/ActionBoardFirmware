#include "PwmPliers.hpp"
#include "Board.hpp"
#include "Logging.hpp"

PwmPliers::PwmPliers(uint8_t id, uint8_t channel, float idleAngle, float activeAngle): Pliers(id, idleAngle, activeAngle),
                                                                                       m_channel(channel){

}

void PwmPliers::init(){
    Logging::println("[PwmPliers] init chan %d", m_channel);
}

void PwmPliers::activate(){
    m_angle = m_activeAngle;
    m_shouldUpdate = true;
}

void PwmPliers::deactivate(){
    m_angle = m_idleAngle;
    m_shouldUpdate = true;
}

void PwmPliers::setAngle(int16_t angle){
    m_angle = angle;
    m_shouldUpdate = true;

}

void PwmPliers::update(){
    Board::Actuators::setPwmServo(m_angle);
    m_shouldUpdate = false;
}
