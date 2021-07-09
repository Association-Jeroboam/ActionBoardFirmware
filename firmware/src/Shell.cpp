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

static void cmd_pliers(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    if (argc == 2) {
        enum pliersID id = (enum pliersID)atoi(argv[0]);
        enum pliersState state;
        if(!strcmp(argv[1], "open")) {
            state = PLIERS_IDLE;
        } else if(!strcmp(argv[1], "close")) {
            state = PLIERS_ACTIVATED;
        } else {
            goto usage;
        }

        if(id > PLIERS_REAR_FAR_LEFT){
            Logging::println("Bad ID");
            goto usage;
        }

        canFrame_t frame = {
            .ID = CAN_PLIERS_ID,
            .len = CAN_PLIERS_LEN,
        };
        frame.data.pliersData = {
            .plierID = id,
            .state = state,
        };
        PliersManager::instance()->processFrame(frame);
    } else {
        goto usage;
    }
    return;

usage:
    Logging::println("usage:");
    Logging::println("pliers [pliersID] [open/close]");
}

static void cmd_pliers_block(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    if (argc == 1) {
        uint8_t state;
        if(!strcmp(argv[0], "engage")) {
            state = 1;
        } else if(!strcmp(argv[0], "disengage")) {
            state = 0;
        } else {
            goto usage;
        }

        canFrame_t frame = {
                .ID = CAN_PLIERS_BLOCK_ID,
                .len = CAN_PLIERS_BLOCK_LEN,
        };
        frame.data.pliersBlockData = {
                .state = state,
        };
        PliersManager::instance()->processFrame(frame);
    } else {
        goto usage;
    }
    return;

usage:
    Logging::println("usage:");
    Logging::println("pliers_block [engage/disengage]");
}

static void cmd_slider(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    (void)chp;
    if (argc == 2) {
        int16_t distance = atoi(argv[1]);
        if(!strcmp(argv[0], "elevator")) {
            Board::Actuators::elevatorSetHeigth(distance);
        } else if(!strcmp(argv[0], "translator")) {
            Logging::println("Not supported yet");
        } else {
            goto usage;
        }
    } else {
        goto usage;
    }
    return;

    usage:
    Logging::println("usage:");
    Logging::println("slider [elevator/translator] [distance (mm)]");
}

static void cmd_arm(BaseSequentialStream* chp, int argc, char* argv[]) {
    (void)chp;
    (void)chp;
    if (argc == 2) {
        uint8_t state = atoi(argv[1]);
        canFrame_t frame = {
                .ID = CAN_ARMS_ID,
                .len = CAN_ARMS_LEN,
        };
        if(!strcmp(argv[0], "left")) {
            frame.data.armData.armID = ARM_LEFT;
        } else if(!strcmp(argv[0], "right")) {
            frame.data.armData.armID = ARM_RIGHT;
        } else {
            goto usage;
        }
        if(state == PLIERS_IDLE) {
            frame.data.armData.state = PLIERS_IDLE;
        } else if (state == PLIERS_ACTIVATED){
            frame.data.armData.state = PLIERS_ACTIVATED;
        } else {
            Logging::println("This arm does not exist");
        }
        PliersManager::instance()->processFrame(frame);
    } else {
        goto usage;
    }
    return;

    usage:
    Logging::println("usage:");
    Logging::println("arm [left/right] [state]");
}

static const ShellCommand commands[] = {
    {"pliers", cmd_pliers},
    {"pliers_block", cmd_pliers_block},
    {"slider", cmd_slider},
    {"arm", cmd_arm},
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
