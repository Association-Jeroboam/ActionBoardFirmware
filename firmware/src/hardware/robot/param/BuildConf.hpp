#pragma once

#include "ch.hpp"
#include "Board.hpp"
#include "board.h"
#include "hal_serial.h"
#include "LocalMath.hpp"
#include "RobotConf.hpp"

#define LOGGING_DRIVER SD2
#define SHELL_DRIVER LOGGING_DRIVER

#define LOGGING_TX_PIN PAL_LINE(GPIOA, 2U)
#define LOGGING_TX_PIN_MODE PAL_MODE_ALTERNATE(7)
#define LOGGING_RX_PIN PAL_LINE(GPIOA, 3U)
#define LOGGING_RX_PIN_MODE PAL_MODE_ALTERNATE(7)

#define LED_2_LINE PAL_LINE(GPIOB, 3U)
#define LED_3_LINE PAL_LINE(GPIOB, 4U)
#define XL320_DRIVER SD1

#define XL320_DATA_PIN      PAL_LINE(GPIOA, 9U)
#define XL320_OLD_DATA_PIN      PAL_LINE(GPIOA, 9U)
#define XL320_DATA_PIN_MODE PAL_MODE_ALTERNATE(7)

#define NUCLEO_LED_LINE      PAL_LINE(GPIOB, 8U)
#define NUCLEO_LED_LINE_MODE PAL_MODE_OUTPUT_PUSHPULL

#define CAN_DRIVER CAND1
#define CAN_TX_PIN      PAL_LINE(GPIOA, 12U)
#define CAN_TX_PIN_MODE PAL_MODE_ALTERNATE(9)
#define CAN_RX_PIN      PAL_LINE(GPIOA, 11U)
#define CAN_RX_PIN_MODE PAL_MODE_ALTERNATE(9)

CANConfig const canConfig = {
        .NBTP = 0x2030C01,	// 1Mbit/s
        .DBTP = 0,
        .CCCR = 0,
        .TEST = 0,
        .RXGFC = 0
};

//I2CConfig const i2cConfig = {
//        .timingr = 0x20A0C4DF,
//        .cr1 = 0,
//        .cr2 = 0,
//};

#define PWM_COUNTING_FREQUENCY 32000000
#define PWM_MOTOR_OUTPUT_FREQUENCY 20000
#define PWM_SERVO_FREQUENCY 3200000
#define PWM_SERVO_OUTPUT_FREQUENCY 50

__extension__ const PWMChannelConfig pwmChannelConf{
	.mode     = PWM_OUTPUT_ACTIVE_HIGH,
	.callback = NULL,
};

__extension__ const PWMConfig pwmValves{
		.frequency = PWM_COUNTING_FREQUENCY,
		.period    = PWM_COUNTING_FREQUENCY / PWM_MOTOR_OUTPUT_FREQUENCY,
		.callback  = NULL,
		.channels  = {
				pwmChannelConf,
				pwmChannelConf,
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
		},
		.cr2  = 0,
		.bdtr = 0,
		.dier = 0,
};

__extension__ const PWMConfig pwmPumps{
		.frequency = PWM_COUNTING_FREQUENCY,
		.period    = PWM_COUNTING_FREQUENCY / PWM_MOTOR_OUTPUT_FREQUENCY,
		.callback  = NULL,
		.channels  = {
				pwmChannelConf,
				pwmChannelConf,
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
		},
		.cr2  = 0,
		.bdtr = 0,
		.dier = 0,
};

__extension__ const PWMConfig pwmServo{
		.frequency = PWM_SERVO_FREQUENCY,
		.period    = PWM_SERVO_FREQUENCY / PWM_SERVO_OUTPUT_FREQUENCY,
		.callback  = NULL,
		.channels  = {
				pwmChannelConf,
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
		},
		.cr2  = 0,
		.bdtr = 0,
		.dier = 0,
};


// PWMs





// Analog
__extension__ const ADCConfig resistanceMeasConf = {
    .difsel = 0,
};
void adc_cb(ADCDriver *adcp);
__extension__ const ADCConversionGroup resistanceMeasConvGroup= {
    .circular = false,
    .num_channels = 1,
    .end_cb = adc_cb,
    .error_cb = NULL,
    .cfgr = ADC_CFGR_DISCEN | ADC_CFGR_DISCNUM_VAL(0) ,
    .cfgr2 = 0,
    .tr1 = 0,
    .tr2 = 0,
    .tr3 = 0,
    .awd2cr = 0,
    .awd3cr = 0,
    .smpr = { 0, 0},
    .sqr = {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN4), 0, 0, 0},
};

#define RESISTANCE_MEAS_DRIVER ADCD2
#define RESISTANCE_MEAS_PIN        PAL_LINE(GPIOA, 7U)
#define RESISTANCE_MEAS_PIN_MODE   PAL_MODE_INPUT_ANALOG

// Power
#define POWER12_ENABLE_PIN      PAL_LINE(GPIOA, 8U)
#define POWER12_ENABLE_PIN_MODE PAL_MODE_UNCONNECTED
#define POWER12_IMON_PIN        PAL_LINE(GPIOA, 0U)
#define POWER12_IMON_PIN_MODE   PAL_MODE_INPUT_ANALOG
#define POWER12_PG_PIN          PAL_LINE(GPIOA, 1U)
#define POWER12_PG_PIN_MODE     PAL_MODE_INPUT
#define POWER12_LDSTR_PIN       PAL_LINE(GPIOA, 4U)
#define POWER12_LDSTR_PIN_MODE  PAL_MODE_OUTPUT_PUSHPULL

#define POWER8_ENABLE_PIN      PAL_LINE(GPIOB, 0U)
#define POWER8_ENABLE_PIN_MODE PAL_MODE_UNCONNECTED
#define POWER8_IMON_PIN        PAL_LINE(GPIOF, 1U)
#define POWER8_IMON_PIN_MODE   PAL_MODE_INPUT_ANALOG
#define POWER8_PG_PIN          PAL_LINE(GPIOA, 10U)
#define POWER8_PG_PIN_MODE     PAL_MODE_INPUT
#define POWER8_LDSTR_PIN       PAL_LINE(GPIOA, 5U)
#define POWER8_LDSTR_PIN_MODE  PAL_MODE_OUTPUT_PUSHPULL