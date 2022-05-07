#include "Board.hpp"
#include "BuildConf.hpp"

const float PUMP_ENABLED_DUTY_CYCLE = 0.99;
const float VALVE_OPENED_DUTY_CYCLE = 0.99;
const float PWM_SERVO_MIN_DC = 0.03;
const float PWM_SERVO_MAX_DC = 0.12;

void setDutyCycle(PWMDriver * pwmd, uint16_t channel, float duty_cycle);

void Board::Actuators::init() {
	palSetLineMode(PUMP_0_PIN, PUMP_0_PIN_MODE);
	palSetLineMode(PUMP_1_PIN, PUMP_1_PIN_MODE);
	palSetLineMode(VALVE_0_PIN, VALVE_0_PIN_MODE);
	palSetLineMode(VALVE_1_PIN, VALVE_1_PIN_MODE);

	palSetLineMode(SERVO_0_PIN, SERVO_0_PIN_MODE);

	pwmStart(&PUMPS_DRIVER,   &pwmPumps);
	pwmStart(&VALVES_DRIVER,  &pwmValves);
	pwmStart(&SERVO_0_DRIVER, &pwmServo);
    setDutyCycle(&SERVO_0_DRIVER, 0, 0.5);
}

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
    setDutyCycle(&PUMPS_DRIVER, valve, dc);
}

void setDutyCycle(PWMDriver * pwmd, uint16_t channel, float duty_cycle) {
	uint16_t percentage = (uint16_t)(duty_cycle * 10000);
	pwmEnableChannel(pwmd,
					 channel,
					 PWM_PERCENTAGE_TO_WIDTH(pwmd, percentage));
}

void Board::Actuators::setPwmServo(uint16_t angle) {
    if(angle >= 360) return;
    float fangle = (float)angle;
    float dc = fangle * (PWM_SERVO_MAX_DC - PWM_SERVO_MIN_DC) / 360. + PWM_SERVO_MIN_DC;
    setDutyCycle(&SERVO_0_DRIVER, 0, dc);
}
