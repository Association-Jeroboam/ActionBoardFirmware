/*
    Copyright (C) 2016 Jonathan Struebel

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    common/shellcfg.c
 * @brief   CLI shell config.
 *
 * @addtogroup SHELL
 * @{
 */

#include <stdlib.h>
#include <cstring>
#include "hal.h"
#include "chprintf.h"
#include "shell.h"
#include "BuildConf.hpp"
#include "Logging.hpp"
#include "CanProtocol.hpp"
#include "PliersManager.hpp"
#include "Board.hpp"

char** endptr;

/*
 * Shell history buffer
 */
char history_buffer[SHELL_MAX_HIST_BUFF];

/*
 * Shell completion buffer
 */
char* completion_buffer[SHELL_MAX_COMPLETIONS];

/*
 * Shell commands
 */


static void cmd_arm(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    Logging::println("usage:");
    Logging::println("arm [left/right] [state]");
    Logging::println("NOT IMPLEMENTED.");
}

static void cmd_pump(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    if (argc == 2) {
        uint8_t state = atoi(argv[1]);
        Board::Actuators::Pump pump;
        if(!strcmp(argv[0], "left")) {
            pump = Board::Actuators::PUMP_LEFT;
        } else if(!strcmp(argv[0], "right")) {
            pump = Board::Actuators::PUMP_RIGHT;
        } else {
            goto usage;
        }
        Board::Actuators::setPumpState(pump, state == 1);
    } else {
        goto usage;
    }
    return;

usage:
    Logging::println("usage:");
    Logging::println("pump [left/right] [0/1]");
}

static void cmd_valve(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    if (argc == 2) {
        uint8_t state = atoi(argv[1]);
        Board::Actuators::Valve valve;
        if(!strcmp(argv[0], "left")) {
            valve = Board::Actuators::VALVE_LEFT;
        } else if(!strcmp(argv[0], "right")) {
            valve = Board::Actuators::VALVE_RIGHT;
        } else {
            goto usage;
        }
        Board::Actuators::setValveState(valve, state == 1);
    } else {
        goto usage;
    }
    return;

usage:
    Logging::println("usage:");
    Logging::println("valve [left/right] [0/1]");
}

static void cmd_servo(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
//#ifdef BLUE_ROBOT
    if (argc == 2) {
        enum servoID servoID = (enum servoID)atoi(argv[0]);
        uint16_t angle = atoi(argv[1]);
        if(angle>300) {
            goto usage;
        }
        constexpr float degToRad = 2 * M_PI / 360;
        Servo* servo = Board::Actuators::getServoByID(servoID);
        if(servo) {
            servo->setAngle(angle * degToRad);
            PliersManager::instance()->updateServos();
        } else {
            goto usage;
        }

    } else {
        goto usage;
    }
    return;
//#endif
usage:
    Logging::println("usage: ON BLUE ROBOT");
    Logging::println("servo [id] [0-300]");
}

static void cmd_reboot(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    (void)argc;
    (void)argv;
    if (argc == 0) {
        NVIC_SystemReset();
    }
}

static void cmd_turbine(BaseSequentialStream* chp, int argc, char* argv[]) {
    using namespace Board::Actuators;
    if (argc == 1) {
        uint8_t speed = atoi(argv[0]);
        enum TurbineSpeed turbineSpeed;
        if(speed == 1) {
            Logging::println("Slow");
            turbineSpeed = TURBINE_SPEED_SLOW;
        } else if(speed == 2) {
            Logging::println("fast");
            turbineSpeed = TURBINE_SPEED_FAST;
        } else {
            Logging::println("Stop");
            turbineSpeed = TURBINE_SPEED_STOPPED;
        }
        setTurbineSpeed(turbineSpeed);
        return;
    }
    Logging::println("usage:");
    Logging::println("turbine [speed] (0-2)");
}

static const ShellCommand commands[] = {
    {"arm", cmd_arm},
    {"pump", cmd_pump},
    {"valve", cmd_valve},
    {"servo", cmd_servo},
    {"reboot", cmd_reboot},
    {"turbine", cmd_turbine},
    {NULL, NULL},
};
/*
 * Shell configuration
 */
ShellConfig shell_cfg = {
    (BaseSequentialStream*)&SHELL_DRIVER,
    commands,
    history_buffer,
    sizeof(history_buffer),
    completion_buffer};

/** @} */
