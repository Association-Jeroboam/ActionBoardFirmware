#include "DxlPliers.hpp"
#include "ch.hpp"
#include "Board.hpp"
#include "Logging.hpp"

DxlPliers::DxlPliers(uint8_t id, float idleAngle, float activeAngle):Pliers(id, idleAngle, activeAngle){}

void DxlPliers::init(){
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Logging::println("[DxlPliers] init %d at %f", m_id, bus->getPresentPosition(m_id, UNIT_DEGREE));

    bus->ledOff(m_id);
    bus->torqueOff(m_id);
    chThdSleepMilliseconds(20);


    // Configure actuator here
    bus->setOperatingMode(m_id, OP_POSITION);
    bus->setPositionPIDGain(m_id, DXL_PLIERS_P_GAIN, DXL_PLIERS_I_GAIN, DXL_PLIERS_D_GAIN);
    bus->setGoalVelocity(m_id, 20., UNIT_PERCENT);
    // Configuration done
    bus->torqueOn(m_id);
    bus->ledOn(m_id, LED_GREEN);
    bus->setTorqueLimit(m_id, DXL_PLIERS_TORQUE_LIMIT, UNIT_PERCENT);
}

void DxlPliers::deactivate() {
    Board::Com::DxlServo::getBus()->setGoalPosition(m_id, m_idleAngle, UNIT_DEGREE);
}

void DxlPliers::activate() {
    Board::Com::DxlServo::getBus()->setGoalPosition(m_id, m_activeAngle, UNIT_DEGREE);
}

void DxlPliers::setAngle(int16_t angle) {
    Board::Com::DxlServo::getBus()->setGoalPosition(m_id, angle, UNIT_DEGREE);
}
