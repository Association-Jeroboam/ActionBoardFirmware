#pragma once

#include  <inttypes.h>


#if defined(WHITE_ROBOT)
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ID  = 19;
constexpr uint8_t PLIERS_FRONT_LEFT_ID      = 2;
constexpr uint8_t PLIERS_FRONT_RIGHT_ID     = 4;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ID = 3;
constexpr uint8_t PLIERS_REAR_FAR_RIGHT_ID  = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_ID      = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_ID     = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_ID       = 0;  //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_ID   = 0;  //TODO: CHANGE ME!


constexpr uint8_t PLIERS_FRONT_FAR_LEFT_IDLE_ANGLE    = 170;
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ACTIVE_ANGLE  = 190;
constexpr uint8_t PLIERS_FRONT_LEFT_IDLE_ANGLE        = 115;
constexpr uint8_t PLIERS_FRONT_LEFT_ACTIVE_ANGLE      = 95;
constexpr uint8_t PLIERS_FRONT_RIGHT_IDLE_ANGLE       = 180;
constexpr uint8_t PLIERS_FRONT_RIGHT_ACTIVE_ANGLE     =     200;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_IDLE_ANGLE   = 130;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ACTIVE_ANGLE = 110;

constexpr uint8_t PLIERS_REAR_FAR_RIGHT_PWM_CHANNEL = 2; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_PWM_CHANNEL = 3; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_PWM_CHANNEL = 1; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_PWM_CHANNEL = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_PWM_CHANNEL = 4; //TODO: CHANGE ME!

constexpr float PLIERS_REAR_FAR_RIGHT_IDLE_ANGLE   = 160;
constexpr float PLIERS_REAR_FAR_RIGHT_ACTIVE_ANGLE = 130;
constexpr float PLIERS_REAR_RIGHT_IDLE_ANGLE       = 150;
constexpr float PLIERS_REAR_RIGHT_ACTIVE_ANGLE     = 0;
constexpr float PLIERS_REAR_MIDDLE_IDLE_ANGLE      = 150;
constexpr float PLIERS_REAR_MIDDLE_ACTIVE_ANGLE    = 0;
constexpr float PLIERS_REAR_LEFT_ANGLE_IDLE        = 150;
constexpr float PLIERS_REAR_LEFT_ACTIVE_ANGLE      = 0;
constexpr float PLIERS_REAR_FAR_LEFT_IDLE_ANGLE    = 150;
constexpr float PLIERS_REAR_FAR_LEFT_ACTIVE_ANGLE  = 0;

constexpr uint8_t PLIERS_BLOCK_LEFT_ID    = 6;
constexpr uint8_t PLIERS_BLOCK_LEFT_IDLE_ANGLE = 80;
constexpr uint8_t PLIERS_BLOCK_LEFT_ACTIVE_ANGLE = 140;

constexpr uint8_t PLIERS_BLOCK_RIGHT_ID   = 5;
constexpr uint8_t PLIERS_BLOCK_RIGHT_IDLE_ANGLE = 220;
constexpr uint8_t PLIERS_BLOCK_RIGHT_ACTIVE_ANGLE = 160;

constexpr uint8_t SLIDER_ELEVATOR_ID = 1;
constexpr float   SLIDER_ELEVATOR_DISTANCE_PER_TURN = 15.3 * 2. * M_PI;


constexpr uint8_t ARM_RIGHT_ID              = 17;
constexpr uint8_t ARM_LEFT_ID               = 12;
constexpr uint8_t FLAG_ID                   = 0;  //TODO CHANGE ME

constexpr float ARM_RIGHT_IDLE_ANGLE   = 150;
constexpr float ARM_RIGHT_ACTIVE_ANGLE = 240;
constexpr float ARM_LEFT_IDLE_ANGLE   = 150;
constexpr float ARM_LEFT_ACTIVE_ANGLE = 60;
constexpr float FLAG_IDLE_ANGLE   = 150;
constexpr float FLAG_ANGLE = 0;

#elif defined(BRUT_ROBOT)
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ID  = 7;
constexpr uint8_t PLIERS_FRONT_LEFT_ID      = 8;
constexpr uint8_t PLIERS_FRONT_RIGHT_ID     = 11;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ID = 10;

constexpr uint8_t PLIERS_FRONT_FAR_LEFT_IDLE_ANGLE    = 170;
constexpr uint8_t PLIERS_FRONT_FAR_LEFT_ACTIVE_ANGLE  = 190;
constexpr uint8_t PLIERS_FRONT_LEFT_IDLE_ANGLE        = 115;
constexpr uint8_t PLIERS_FRONT_LEFT_ACTIVE_ANGLE      = 95;
constexpr uint8_t PLIERS_FRONT_RIGHT_IDLE_ANGLE       = 180;
constexpr uint8_t PLIERS_FRONT_RIGHT_ACTIVE_ANGLE     =     200;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_IDLE_ANGLE   = 130;
constexpr uint8_t PLIERS_FRONT_FAR_RIGHT_ACTIVE_ANGLE = 110;

constexpr uint8_t PLIERS_REAR_FAR_RIGHT_ID  = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_RIGHT_ID      = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_MIDDLE_ID     = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_LEFT_ID       = 0; //TODO: CHANGE ME!
constexpr uint8_t PLIERS_REAR_FAR_LEFT_ID   = 0; //TODO: CHANGE ME!

constexpr uint8_t ARM_RIGHT_ID              = 17;
constexpr uint8_t ARM_LEFT_ID               = 12;
constexpr uint8_t FLAG_ID                   = 7;

constexpr float ARM_RIGHT_IDLE_ANGLE   = 150;
constexpr float ARM_RIGHT_ACTIVE_ANGLE = 240;
constexpr float ARM_LEFT_IDLE_ANGLE   = 150;
constexpr float ARM_LEFT_ACTIVE_ANGLE = 60;
constexpr float FLAG_IDLE_ANGLE   = 150;
constexpr float FLAG_ANGLE = 0;

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
