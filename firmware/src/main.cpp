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
#include "Resistance_0_1.h"
#include "PeripheralManager.hpp"
#include "ResistanceMeasure.hpp"

static THD_WORKING_AREA(waShellThread, SHELL_WA_SIZE);

static PeripheralManager PeripheralManager;

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

void publishResistanceMeasure(float resistance) {
    static CanardTransferID transfer_id = 0;
    jeroboam_datatypes_sensors_Resistance_0_1 pubResistance;

    pubResistance.value.value = resistance;
    size_t buf_size = jeroboam_datatypes_sensors_Resistance_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;
    uint8_t buffer[jeroboam_datatypes_sensors_Resistance_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

    jeroboam_datatypes_sensors_Resistance_0_1_serialize_(&pubResistance, buffer, &buf_size);

    CanardTransferMetadata metadata = {
            .priority = CanardPriorityNominal,
            .transfer_kind = CanardTransferKindMessage,
            .port_id = ACTION_RESISTANCE_MEAS_ID,
            .remote_node_id = CANARD_NODE_ID_UNSET,
            .transfer_id = transfer_id,

    };

    Board::Com::CANBus::send(&metadata, buf_size, buffer);

    transfer_id++;
}


int main() {
    halInit();
    chSysInit();

    Logging::init();
    Logging::println("Starting up");
    shellInit();
    Board::init();

    chThdSleepMilliseconds(20);
    // PliersManager::instance()->start(NORMALPRIO);

    PeripheralManager.init();

    chThdCreateStatic(waShellThread, sizeof(waShellThread), NORMALPRIO,
                      shellThread, (void*)&shell_cfg);


    uint32_t count = 0;
    while (!chThdShouldTerminateX()) {
        count++;
        if(count % 10 == 0) {
            count = 0;
            // Board::IO::toggleNucleoLed();
            cyphalHeartBeatRoutine();
        }

        chThdSleepMilliseconds(100);
    }
    Logging::println("Shutting down");
}
