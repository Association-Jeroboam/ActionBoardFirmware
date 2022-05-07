#include "Board.hpp"
#include "BuildConf.hpp"

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

	setDutyCycle(&PUMPS_DRIVER, 0, 0.5);
	setDutyCycle(&PUMPS_DRIVER, 1, 0.5);
	setDutyCycle(&VALVES_DRIVER, 0, 0.5);
	setDutyCycle(&VALVES_DRIVER, 1, 0.5);
	setDutyCycle(&SERVO_0_DRIVER, 0, 0.5);

}

void setDutyCycle(PWMDriver * pwmd, uint16_t channel, float duty_cycle) {
	uint16_t percentage = (uint16_t)(duty_cycle * 10000);
	pwmEnableChannel(pwmd,
					 channel,
					 PWM_PERCENTAGE_TO_WIDTH(pwmd, percentage));
}
