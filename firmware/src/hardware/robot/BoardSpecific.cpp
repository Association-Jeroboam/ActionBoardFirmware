#include "Board.hpp"
#include "BuildConf.hpp"
#include "RobotConf.hpp"

const float PUMP_ENABLED_DUTY_CYCLE = 0.99;
const float VALVE_OPENED_DUTY_CYCLE = 0.99;
const float PWM_SERVO_MIN_DC = 0.02;
const float PWM_SERVO_MAX_DC = 0.12;

void setDutyCycle(PWMDriver * pwmd, uint16_t channel, float duty_cycle);

void Board::Actuators::setPumpState(enum Pump pump, bool enabled) {
    float dc;
    if(enabled) {
        dc = PUMP_ENABLED_DUTY_CYCLE;
    } else {
        dc = 0.;
    }
    setDutyCycle(&PUMPS_DRIVER, pump, dc);
}

void Board::Actuators::setValveState(enum Valve valve, bool opened) {
    float dc;
    if(opened) {
        dc = VALVE_OPENED_DUTY_CYCLE;
    } else {
        dc = 0.;
    }
    setDutyCycle(&VALVES_DRIVER, valve, dc);
}

void setDutyCycle(PWMDriver * pwmd, uint16_t channel, float duty_cycle) {
	uint16_t percentage = (uint16_t)(duty_cycle * 10000);
	pwmEnableChannel(pwmd,
					 channel,
					 PWM_PERCENTAGE_TO_WIDTH(pwmd, percentage));
}

void Board::Actuators::setPwmServo(uint16_t angle) {
    if( angle > PWM_PLIERS_MAX_ANGLE || angle < PWM_PLIERS_MIN_ANGLE) return;
    float fangle = (float)angle;
    float dc = fangle * (PWM_SERVO_MAX_DC - PWM_SERVO_MIN_DC) / 180. + PWM_SERVO_MIN_DC;
    setDutyCycle(&SERVO_0_DRIVER, 0, dc);
}
