#include <cstring>
#include "DxlPliers.hpp"
#include "ch.hpp"
#include "Board.hpp"
#include "Logging.hpp"

DxlPliers::DxlPliers(uint8_t id, float idleAngle, float activeAngle):
Pliers(id, idleAngle, activeAngle),
m_config()
{
    m_config = (DxlPliersConfig){
        .torqueLimit = DXL_PLIERS_TORQUE_LIMIT,
        .movingSpeed = DXL_PLIERS_MOVING_SPEED_PERCENT,
        .p = DXL_PLIERS_P_GAIN,
        .i = DXL_PLIERS_I_GAIN,
        .d = DXL_PLIERS_D_GAIN,
        .color = LedColor::LED_GREEN,
    };
}

void DxlPliers::init(){
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Logging::println("[DxlPliers] init %d at %f", m_id, bus->getPresentPosition(m_id, UNIT_DEGREE));
    Board::Com::DxlServo::lockBus();
    bus->ledOff(m_id);
    bus->torqueOff(m_id);
    Board::Com::DxlServo::unlockBus();
    chThdSleepMilliseconds(20);

    reset();
    deactivate();
}

void DxlPliers::update() {
    const float radToRAw = 1023 / DXL_PLIERS_MAX_ANGLE_RAD; // (1/DXL_PLIERS_MAX_ANGLE_RAD) * 1023 maps input [0 - DXL_PLIERS_MAX_ANGLE_RAD] to [0 - 1023]
    Board::Com::DxlServo::lockBus();
    Board::Com::DxlServo::getBus()->setGoalPosition(m_id, m_angle * radToRAw, UNIT_RAW);
    Board::Com::DxlServo::unlockBus();
    m_shouldUpdate = false;
}

void DxlPliers::updateConfig() {
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Board::Com::DxlServo::lockBus();
    bus->torqueOff(m_id);
    bus->setPositionPIDGain(m_id, m_config.p, m_config.i, m_config.d);
    bus->setTorqueLimit(m_id, m_config.torqueLimit, UNIT_PERCENT);
    bus->setGoalVelocity(m_id, m_config.movingSpeed, UNIT_PERCENT); //TODO check unit conversion
    bus->ledOn(m_id, m_config.color);
    bus->torqueOn(m_id);
    Board::Com::DxlServo::unlockBus();
    m_shouldUpdateConfig = false;
}

void DxlPliers::deactivate() {
    m_angle = m_idleAngle;
    m_shouldUpdate = true;
}

void DxlPliers::activate() {
    m_angle = m_activeAngle;
    m_shouldUpdate = true;
}

void DxlPliers::setAngle(float angle) {
    if(angle != m_angle){
        if(angle > DXL_PLIERS_MAX_ANGLE_RAD || angle < 0) {
            Logging::println("[DxlPliers %u] Angle out of range %f > %f", m_angle, DXL_PLIERS_MAX_ANGLE_RAD);
        }
        m_angle = angle;
        m_shouldUpdate = true;
    }
}

float DxlPliers::getAngle() {
    float angleRaw;
    const float rawToRad = DXL_PLIERS_MAX_ANGLE_RAD / 1023;
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Board::Com::DxlServo::lockBus();
    angleRaw = bus->getPresentPosition(m_id, UNIT_RAW);
    Board::Com::DxlServo::unlockBus();
    return angleRaw * rawToRad;
}

void DxlPliers::setConfig(ServoConfig config) {

    if(memcmp(&config.dxlPliers, &m_config, sizeof(DxlPliersConfig))) {
        m_config = config.dxlPliers;
        m_shouldUpdateConfig = true;
    }
}

void DxlPliers::reset() {
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Board::Com::DxlServo::lockBus();
    bus->setOperatingMode(m_id, OP_POSITION);
    updateConfig();
    // Configuration done
    bus->torqueOn(m_id);
    Board::Com::DxlServo::unlockBus();
}

struct DxlPliersStatus DxlPliers::getSatus() {
    DxlPliersStatus status;
    Dynamixel2Arduino * bus = Board::Com::DxlServo::getBus();
    Board::Com::DxlServo::lockBus();
    //TODO check units
    //TODO add remaining fields to lib
    status.angle = bus->getPresentPosition(m_id, UNIT_DEGREE);
    status.speed = bus->getPresentVelocity(m_id, UNIT_PERCENT);
    status.load  = bus->getPresentCurrent(m_id, UNIT_PERCENT);
    Board::Com::DxlServo::unlockBus();
    return status;
}
