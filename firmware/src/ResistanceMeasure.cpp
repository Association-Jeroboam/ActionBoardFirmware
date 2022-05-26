#include "Logging.hpp"
#include "Board.hpp"
#include "ResistanceMeasure.hpp"
#include "Resistance_0_1.h"

ResistanceMeasure::ResistanceMeasure():
chibios_rt::BaseStaticThread<RESISTANCE_MEAS_WA>()
{

}

void ResistanceMeasure::main() {
    setName("Resistance Measure");
    Logging::println("kjhkjhfkjhkjhkdsj");

    while (true) {
        Logging::println("poil");
        Board::IO::getResistanceMeasure();
//        publishResistanceMeasure(Board::IO::getResistanceMeasure());
        chThdSleepMilliseconds(RESISTANCE_MEAS_UPDATE_MS    );
    }
}



void ResistanceMeasure::