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
    Logging::println("[PwmPliers] activate %d", m_channel);
//    Board::Actuators::setPwmServo(m_channel, m_activeAngle);
}

void PwmPliers::deactivate(){
    Logging::println("[PwmPliers] deactivate %d", m_channel);
//    Board::Actuators::setPwmServo(m_channel, m_idleAngle);
}

void PwmPliers::setAngle(int16_t angle){
    Logging::println("[PwmPliers] setAngle %d %d", angle, m_channel);
//    Board::Actuators::setPwmServo(m_channel, angle);
}
