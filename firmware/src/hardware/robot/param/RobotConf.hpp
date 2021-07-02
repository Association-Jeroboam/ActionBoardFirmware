#pragma once

#include  <inttypes.h>


#if defined(WHITE_ROBOT)
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ID  = 1;
constexpr uint8_t PLIERS_FRONT_LEFT_ID      = 2;
constexpr uint8_t PLIERS_FRONT_RIGHT_ID     = 4;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ID = 3;
constexpr uint8_t PLIERS_REAR_FAR_RIGHT_ID  = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_ID      = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_ID     = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_ID       = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_ID   = 0;  //TODO: CHANGE ME!

constexpr uint8_t PLIERS_REAR_FAR_RIGHT_PWM_CHANNEL = 2; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_PWM_CHANNEL = 3; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_PWM_CHANNEL = 1; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_PWM_CHANNEL = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_PWM_CHANNEL = 4; //TODO: CHANGE ME!

constexpr float PLIERS_REAR_FAR_RIGHT_IDLE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_RIGHT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_IDLE_ANGLE = 250; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_ACTIVE_ANGLE = 150; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ANGLE_IDLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!

constexpr uint8_t PLIERS_BLOCK_LEFT_ID    = 18;
constexpr uint8_t PLIERS_BLOCK_LEFT_IDLE_ANGLE = 76;
constexpr uint8_t PLIERS_BLOCK_LEFT_ACTIVE_ANGLE = 152;

constexpr uint8_t PLIERS_BLOCK_RIGHT_ID   = 24;
constexpr uint8_t PLIERS_BLOCK_RIGHT_IDLE_ANGLE = 224;
constexpr uint8_t PLIERS_BLOCK_RIGHT_ACTIVE_ANGLE = 148;

constexpr uint8_t SLIDER_ELEVATOR_ID = 1;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI;

#elif defined(BRUT_ROBOT)
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ID  = 9;
constexpr uint8_t PLIERS_FRONT_LEFT_ID      = 8;
constexpr uint8_t PLIERS_FRONT_RIGHT_ID     = 11;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ID = 10;
constexpr uint8_t PLIERS_REAR_FAR_RIGHT_ID  = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_ID      = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_ID     = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_ID       = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_ID   = 0; //TODO: CHANGE ME!

constexpr uint8_t PLIERS_REAR_FAR_RIGHT_PWM_CHANNEL = 2; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_PWM_CHANNEL = 3; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_PWM_CHANNEL = 1; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_PWM_CHANNEL = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_PWM_CHANNEL = 4; //TODO: CHANGE ME!

constexpr float PLIERS_REAR_FAR_RIGHT_IDLE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_RIGHT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_RIGHT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_IDLE_ANGLE = 250; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_MIDDLE_ACTIVE_ANGLE = 150; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ANGLE_IDLE = 220; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_LEFT_ACTIVE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_IDLE_ANGLE = 75; //TODO: CHANGE ME!
constexpr float PLIERS_REAR_FAR_LEFT_ACTIVE_ANGLE = 220; //TODO: CHANGE ME!

constexpr uint8_t PLIERS_BLOCK_LEFT_ID    = 6;
constexpr uint8_t PLIERS_BLOCK_LEFT_IDLE_ANGLE = 224;
constexpr uint8_t PLIERS_BLOCK_LEFT_ACTIVE_ANGLE = 148;

constexpr uint8_t PLIERS_BLOCK_RIGHT_ID   = 5;
constexpr uint8_t PLIERS_BLOCK_RIGHT_IDLE_ANGLE = 76;
constexpr uint8_t PLIERS_BLOCK_RIGHT_ACTIVE_ANGLE = 152;

constexpr uint8_t SLIDER_ELEVATOR_ID = 1;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI;

#else
#error "you must define a robot"
#endif /* defined(WHITE_ROBOT) */
