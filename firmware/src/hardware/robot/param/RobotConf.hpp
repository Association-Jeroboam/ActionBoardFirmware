#pragma once

#include  <inttypes.h>

constexpr uint16_t PWM_PLIERS_MIN_ANGLE = 15;
constexpr uint16_t PWM_PLIERS_MAX_ANGLE = 90;
constexpr uint16_t PWM_PLIERS_INIT_ANGLE = 40;

#if defined(RED_ROBOT)

constexpr uint8_t SERVO_ARM_LEFT_A_ID  = 16;
constexpr uint8_t SERVO_ARM_LEFT_B_ID  = 14;
constexpr uint8_t SERVO_ARM_LEFT_C_ID  = 22;
constexpr uint8_t SERVO_ARM_LEFT_D_ID  = 1;
constexpr uint8_t SERVO_ARM_LEFT_E_ID  = 8;
constexpr uint8_t SERVO_ARM_RIGHT_A_ID = 3;
constexpr uint8_t SERVO_ARM_RIGHT_B_ID = 4;
constexpr uint8_t SERVO_ARM_RIGHT_C_ID = 10;
constexpr uint8_t SERVO_ARM_RIGHT_D_ID = 9;
constexpr uint8_t SERVO_ARM_RIGHT_E_ID = 11;

constexpr uint8_t SERVO_RAKE_LEFT_TOP_ID = 5;
constexpr uint8_t SERVO_RAKE_LEFT_BOTTOM_ID = 7;
constexpr uint8_t SERVO_RAKE_RIGHT_TOP_ID = 18;
constexpr uint8_t SERVO_RAKE_RIGHT_BOTTOM_ID = 15;

constexpr uint16_t SERVO_ARM_LEFT_A_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_B_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_C_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_D_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_E_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_A_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_B_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_C_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_D_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_E_IDLE_ANGLE = 180;  //TODO: CHANGE ME!

constexpr uint16_t SERVO_RAKE_LEFT_TOP_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_LEFT_BOTTOM_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_RIGHT_TOP_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_RIGHT_BOTTOM_IDLE_ANGLE = 180; //TODO: CHANGE ME!

constexpr uint8_t LEFT_SLIDER_ID = 101;
constexpr uint8_t RIGHT_SLIDER_ID = 100;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI; //TODO: CHANGE ME!

constexpr uint8_t PWM_PLIERS_CHANNEL_NUMBER = 0;

#elif defined(BLUE_ROBOT)

constexpr uint8_t SERVO_ARM_LEFT_A_ID  = 16;
constexpr uint8_t SERVO_ARM_LEFT_B_ID  = 14;
constexpr uint8_t SERVO_ARM_LEFT_C_ID  = 22;
constexpr uint8_t SERVO_ARM_LEFT_D_ID  = 1;
constexpr uint8_t SERVO_ARM_LEFT_E_ID  = 8;
constexpr uint8_t SERVO_ARM_RIGHT_A_ID = 3;
constexpr uint8_t SERVO_ARM_RIGHT_B_ID = 4;
constexpr uint8_t SERVO_ARM_RIGHT_C_ID = 10;
constexpr uint8_t SERVO_ARM_RIGHT_D_ID = 9;
constexpr uint8_t SERVO_ARM_RIGHT_E_ID = 11;

constexpr uint8_t SERVO_RAKE_LEFT_TOP_ID = 5;
constexpr uint8_t SERVO_RAKE_LEFT_BOTTOM_ID = 7;
constexpr uint8_t SERVO_RAKE_RIGHT_TOP_ID = 18;
constexpr uint8_t SERVO_RAKE_RIGHT_BOTTOM_ID = 15;

constexpr uint16_t SERVO_ARM_LEFT_A_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_B_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_C_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_D_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_LEFT_E_IDLE_ANGLE  = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_A_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_B_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_C_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_D_IDLE_ANGLE = 180;  //TODO: CHANGE ME!
constexpr uint16_t SERVO_ARM_RIGHT_E_IDLE_ANGLE = 180;  //TODO: CHANGE ME!

constexpr uint16_t SERVO_RAKE_LEFT_TOP_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_LEFT_BOTTOM_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_RIGHT_TOP_IDLE_ANGLE = 180; //TODO: CHANGE ME!
constexpr uint16_t SERVO_RAKE_RIGHT_BOTTOM_IDLE_ANGLE = 180; //TODO: CHANGE ME!

constexpr uint8_t LEFT_SLIDER_ID = 101;
constexpr uint8_t RIGHT_SLIDER_ID = 100;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI; //TODO: CHANGE ME!

constexpr uint8_t PWM_PLIERS_CHANNEL_NUMBER = 0;

#else
#error "you must define a robot"
#endif /* defined(RED_ROBOT) */
