#include <cstring>
#include "Slider.hpp"
#include "Board.hpp"
#include "BuildConf.hpp"
#include "RobotConf.hpp"
#include "ch.hpp"
#include "Logging.hpp"

Slider::Slider(uint8_t id):
Servo(id),
m_position(0){}

void Slider::init(){
    Logging::println("[Slider] init %d", m_id);
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    bus->ledOff(m_id);
    bus->torqueOff(m_id);
    chThdSleepMilliseconds(20);

    // Configure actuator here
    bus->setOperatingMode(m_id, OP_EXTENDED_POSITION);
    // Configuration done
    bus->torqueOn(m_id);
    bus->ledOn(m_id);
    float pos = bus->getPresentPosition(m_id, UNIT_DEGREE);
    Logging::println("pos %f", pos);
}

void Slider::goToPosition(int16_t position){
    if (m_position != position) {
        m_position = position;
        m_shouldUpdate = true;
    }

}

void Slider::setAngle(float angle) {
    m_position = angle * SLIDER_ELEVATOR_DISTANCE_PER_TURN / 360.;
    m_shouldUpdate = true;
}

void Slider::setConfig(ServoConfig config) {

    if(memcmp(&config.sliderConfig, &m_config, sizeof(DxlPliersConfig))) {
        m_config = config.sliderConfig;
        m_shouldUpdateConfig = true;
    }
}

void Slider::setPIDGains(uint16_t p, uint16_t i, uint16_t d){
    m_config.position_p = p;
    m_config.position_i = i;
    m_config.position_d = d;
    m_shouldUpdateConfig = true;
}

void Slider::update(){
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    bus->setGoalPosition(m_id, (float)m_position / SLIDER_ELEVATOR_DISTANCE_PER_TURN * 360., UNIT_DEGREE);
    m_shouldUpdate = false;
}

void Slider::updateConfig(){
    Board::Com::DxlServo::lockBus();
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    bus->torqueOff(m_id);
    bus->setPositionPIDGain(m_id, m_config.position_p, m_config.position_i, m_config.position_d);
    bus->torqueOn(m_id);
    Board::Com::DxlServo::unlockBus();
    m_shouldUpdateConfig = false;
}
