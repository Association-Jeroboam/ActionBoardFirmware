#include <ch.hpp>
#include <hal.h>
#include <shell.h>
#include "Shell.hpp"
#include "Board.hpp"
#include "Logging.hpp"
#include "BuildConf.hpp"
#include "Dynamixel2Arduino.h"
#include "PliersManager.hpp"
//#include "PCA9635.h"
//#include "Adafruit_PWMServoDriver.h"
#include "hal_pal.h"

static THD_WORKING_AREA(waShellThread, SHELL_WA_SIZE);

int main() {
    halInit();
    chSysInit();

    Logging::init();
    Logging::println("Starting up");
    shellInit();
    Board::init();

//    PliersManager::instance()->start(NORMALPRIO);
    chThdCreateStatic(waShellThread, sizeof(waShellThread), NORMALPRIO,
                      shellThread, (void*)&shell_cfg);
    chThdSleepMilliseconds(20);

    uint8_t id = 4;
    while (!chThdShouldTerminateX()) {
        Board::IO::toggleNucleoLed();
        Board::IO::getResistanceMeasure();
        chThdSleepMilliseconds(100);
    }
    Logging::println("Shutting down");
}
