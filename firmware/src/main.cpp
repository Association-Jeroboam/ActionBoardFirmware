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
#include "Heartbeat_1_0.h"

static THD_WORKING_AREA(waShellThread, SHELL_WA_SIZE);

void cyphalHeartBeatRoutine() {
    static CanardTransferID transfer_id = 0;
    static uint16_t MSB = 0;
    static uint32_t before = 0;
    uint32_t now = chVTGetSystemTime();
    if(now <= before) {
        MSB++;
    }
    const uavcan_node_Heartbeat_1_0 heartbeat = {
            .uptime = TIME_I2S(now | (MSB << 16)),
            .health = {
                    .value = uavcan_node_Health_1_0_NOMINAL
            },
            .mode = {
                    .value = uavcan_node_Mode_1_0_OPERATIONAL,
            },
            .vendor_specific_status_code = 42,
    };

    size_t buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

    uavcan_node_Heartbeat_1_0_serialize_(&heartbeat, buffer, &buf_size);


    const CanardTransferMetadata metadata = {
            .priority = CanardPriorityNominal,
            .transfer_kind = CanardTransferKindMessage,
            .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
            .remote_node_id = CANARD_NODE_ID_UNSET,
            .transfer_id = transfer_id,
    };
    transfer_id++;
    Board::Com::CANBus::send(&metadata, buf_size,  buffer);
    before = now;
}

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
        cyphalHeartBeatRoutine();
        chThdSleepMilliseconds(1000);
    }
    Logging::println("Shutting down");
}
