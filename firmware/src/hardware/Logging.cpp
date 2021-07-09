#include "Logging.hpp"
#include "Board.hpp"
#include <hal.h>

#include <chprintf.h>
#include <cstdarg>
#include "BuildConf.hpp"
#include "hal_serial.h"

bool disablePrint = false;

__extension__ SerialConfig serialConfig {
        .speed = 115200,
        .cr1 = 0,
        .cr2 = USART_CR2_STOP1_BITS,
        .cr3 = 0
};

void Logging::init() {
    palSetLineMode(LOGGING_TX_PIN, LOGGING_TX_PIN_MODE);
    palSetLineMode(LOGGING_RX_PIN, LOGGING_RX_PIN_MODE);
    sdStart(&LOGGING_DRIVER, &serialConfig);
}

void Logging::print(const char* fmt, ...) {
    if(disablePrint) return;
    va_list args;
    va_start(args, fmt);
    chvprintf(reinterpret_cast<BaseSequentialStream *>(&LOGGING_DRIVER), fmt, args);
    va_end(args);
}

void Logging::println(const char* fmt, ...) {
    if(disablePrint) return;
    va_list args;
    va_start(args, fmt);
    chvprintf(reinterpret_cast<BaseSequentialStream *>(&LOGGING_DRIVER), fmt, args);
    chprintf(reinterpret_cast<BaseSequentialStream *>(&LOGGING_DRIVER), "\n");
    va_end(args);
}

void Logging::setDisablePrint(bool disable) {
    disablePrint = disable;
}

void Logging::write(uint8_t *txBuffer, uint16_t len) {
    sdWrite(&LOGGING_DRIVER, txBuffer, len);
}
