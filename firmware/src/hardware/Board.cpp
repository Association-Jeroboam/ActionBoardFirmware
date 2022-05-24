#include "Board.hpp"
#include "board.h"
#include "ch.hpp"
#include "hal.h"
#include "BuildConf.hpp"
#include "Logging.hpp"
#include "CanRxThread.hpp"
#include "CanTxThread.hpp"
#include "Pliers.hpp"
#include "DxlPliers.hpp"
#include "Slider.hpp"
#include "Pliers.hpp"
#include "PwmPliers.hpp"
#include "canard.h"
#include "CanProtocol.hpp"

chibios_rt::Mutex dxlBusMutex;

inline void* canardSpecificHeapAlloc(CanardInstance* ins, size_t amount) {
    (void)ins;
    return chHeapAlloc(NULL, amount);
}

inline void canardSpecificHeapFree(CanardInstance* ins, void* pointer) {
    (void)ins;
    if(pointer) chHeapFree(pointer);
}

CanardInstance canardInstance;
CanRxThread canRxThread(&canardInstance);
CanTxThread canTxThread(&canardInstance);

static DxlPliers s_armLeftA(SERVO_ARM_LEFT_A_ID,   SERVO_ARM_LEFT_A_IDLE_ANGLE, SERVO_ARM_LEFT_A_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armLeftB(SERVO_ARM_LEFT_B_ID,   SERVO_ARM_LEFT_B_IDLE_ANGLE, SERVO_ARM_LEFT_B_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armLeftC(SERVO_ARM_LEFT_C_ID,   SERVO_ARM_LEFT_C_IDLE_ANGLE, SERVO_ARM_LEFT_C_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armLeftD(SERVO_ARM_LEFT_D_ID,   SERVO_ARM_LEFT_D_IDLE_ANGLE, SERVO_ARM_LEFT_D_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armLeftE(SERVO_ARM_LEFT_E_ID,   SERVO_ARM_LEFT_E_IDLE_ANGLE, SERVO_ARM_LEFT_E_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armRightA(SERVO_ARM_RIGHT_A_ID, SERVO_ARM_RIGHT_A_IDLE_ANGLE, SERVO_ARM_RIGHT_A_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armRightB(SERVO_ARM_RIGHT_B_ID, SERVO_ARM_RIGHT_B_IDLE_ANGLE, SERVO_ARM_RIGHT_B_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armRightC(SERVO_ARM_RIGHT_C_ID, SERVO_ARM_RIGHT_C_IDLE_ANGLE, SERVO_ARM_RIGHT_C_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armRightD(SERVO_ARM_RIGHT_D_ID, SERVO_ARM_RIGHT_D_IDLE_ANGLE, SERVO_ARM_RIGHT_D_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_armRightE(SERVO_ARM_RIGHT_E_ID, SERVO_ARM_RIGHT_E_IDLE_ANGLE, SERVO_ARM_RIGHT_E_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_rakeLeftTop(SERVO_RAKE_LEFT_TOP_ID, SERVO_RAKE_LEFT_TOP_IDLE_ANGLE, SERVO_RAKE_LEFT_TOP_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_rakeLeftBottom(SERVO_RAKE_LEFT_BOTTOM_ID, SERVO_RAKE_LEFT_BOTTOM_IDLE_ANGLE, SERVO_RAKE_LEFT_BOTTOM_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_rakeRightTop(SERVO_RAKE_RIGHT_TOP_ID, SERVO_RAKE_RIGHT_TOP_IDLE_ANGLE, SERVO_RAKE_RIGHT_TOP_IDLE_ANGLE + M_PI/4); //TODO change active angle
static DxlPliers s_rakeRightBottom(SERVO_RAKE_RIGHT_BOTTOM_ID, SERVO_RAKE_RIGHT_BOTTOM_IDLE_ANGLE, SERVO_RAKE_RIGHT_BOTTOM_IDLE_ANGLE + M_PI/4); //TODO change active angle

static Slider s_leftSlider(LEFT_SLIDER_ID);
static Slider s_rightSlider(RIGHT_SLIDER_ID);

constexpr uint32_t DXL_BAUDRATE = 1000000;
Dynamixel2Arduino * dxlBus;


void Board::init() {
    Board::IO::init();
    Board::Actuators::init();
    Board::Com::init();

}

void Board::Com::init() {
    Board::Com::CANBus::init();
//    Board::Com::I2CBus::init();
    Board::Com::DxlServo::init();
}

void Board::Com::CANBus::init(){
    palSetLineMode(CAN_TX_PIN, CAN_TX_PIN_MODE);
    palSetLineMode(CAN_RX_PIN, CAN_RX_PIN_MODE);
    canStart(&CAN_DRIVER, &canConfig);
    canardInstance = canardInit(canardSpecificHeapAlloc, canardSpecificHeapFree);
    canardInstance.node_id = CAN_PROTOCOL_ACTION_BOARD_ID;
    canTxThread.start(NORMALPRIO+1);
    canRxThread.start(NORMALPRIO+2);
    // let Threads finish initialization
    chThdYield();
}

bool Board::Com::CANBus::send(const CanardTransferMetadata* const metadata,
                              const size_t                        payload_size,
                              const void* const                   payload) {
    return canTxThread.send(metadata, payload_size, payload);
}

void Board::Com::CANBus::registerCanMsg(CanListener *listener,
                                        CanardTransferKind transfer_kind,
                                        CanardPortID port_id,
                                        size_t extent) {
    canRxThread.subscribe(listener, transfer_kind, port_id, extent);
}

void Board::Com::DxlServo::init(){
    palSetLineMode(XL320_DATA_PIN, XL320_DATA_PIN_MODE);
    dxlBus = new Dynamixel2Arduino(&XL320_DRIVER);
    dxlBus->begin(DXL_BAUDRATE);
    dxlBus->setPortProtocolVersion(2.0);

    s_armLeftA.init();
    s_armLeftB.init();
    s_armLeftC.init();
    s_armLeftD.init();
    s_armLeftE.init();
    s_armRightA.init();
    s_armRightB.init();
    s_armRightC.init();
    s_armRightD.init();
    s_armRightE.init();
    s_rakeLeftTop.init();
    s_rakeLeftBottom.init();
    s_rakeRightTop.init();
    s_rakeRightBottom.init();
    s_leftSlider.init();
    s_rightSlider.init();

}

void Board::Com::DxlServo::lockBus(){
//    dxlBusMutex.lock();
}

void Board::Com::DxlServo::unlockBus(){
//    dxlBusMutex.unlock();
}

Dynamixel2Arduino * Board::Com::DxlServo::getBus(){
    return dxlBus;
}

Servo*  Board::Actuators::getServoByID(enum servoID ID){

    switch (ID) {
        case SERVO_ARM_LEFT_A: return &s_armLeftA;
        case SERVO_ARM_LEFT_B: return &s_armLeftB;
        case SERVO_ARM_LEFT_C: return &s_armLeftC;
        case SERVO_ARM_LEFT_D: return &s_armLeftD;
        case SERVO_ARM_LEFT_E: return &s_armLeftE;
        case SERVO_ARM_RIGHT_A: return &s_armRightA;
        case SERVO_ARM_RIGHT_B: return &s_armRightB;
        case SERVO_ARM_RIGHT_C: return &s_armRightC;
        case SERVO_ARM_RIGHT_D: return &s_armRightD;
        case SERVO_ARM_RIGHT_E: return &s_armRightE;
        case SERVO_RAKE_LEFT_TOP: return &s_rakeLeftTop;
        case SERVO_RAKE_LEFT_BOTTOM: return &s_rakeLeftBottom;
        case SERVO_RAKE_RIGHT_TOP: return &s_rakeRightTop;
        case SERVO_RAKE_RIGHT_BOTTOM: return &s_rakeRightBottom;
        case LEFT_SLIDER: return &s_leftSlider;
        case RIGHT_SLIDER: return &s_rightSlider;
    }
    return nullptr;
}

void Board::Actuators::elevatorSetHeigth(int16_t height) {
//    s_elevator.goToDistance(height);
}

void Board::Com::I2CBus::init(){
//    palSetLineMode(I2C_SERVO_SCL_PIN, I2C_SERVO_SCL_PIN_MODE);
//    palSetLineMode(I2C_SERVO_SDA_PIN, I2C_SERVO_SDA_PIN_MODE);
//    i2cStart(&I2C_SERVO_DRIVER, &i2cConfig);

}

bool Board::Com::I2CBus::transmit(uint8_t addr, uint8_t *txData, uint8_t txLen, uint8_t *rxData, uint8_t rxLen){
    (void)addr;
    (void)txData;
    (void)txLen;
    (void)rxData;
    (void)rxLen;

//    i2cAcquireBus(&I2C_SERVO_DRIVER);
//    msg_t ret = i2cMasterTransmitTimeout(&I2C_SERVO_DRIVER, addr, txData, txLen, rxData, rxLen, TIME_MS2I(10));
//    i2cReleaseBus(&I2C_SERVO_DRIVER);
//    return ret == MSG_OK;
	return false;
}

bool Board::Com::I2CBus::receive(uint8_t addr, uint8_t *rxData, uint8_t rxLen){
    (void)addr;
    (void)rxData;
    (void)rxLen;
//    i2cAcquireBus(&I2C_SERVO_DRIVER);(void);
//    msg_t ret = i2cMasterReceiveTimeout(&I2C_SERVO_DRIVER, addr, rxData, rxLen, TIME_MS2I(10));
//    i2cReleaseBus(&I2C_SERVO_DRIVER);
//
//    return ret == MSG_OK;
	return false;
}

void Board::IO::init(){

    palSetLineMode(NUCLEO_LED_LINE, NUCLEO_LED_LINE_MODE);
//    palSetLineMode(LED_2_LINE,      PAL_MODE_OUTPUT_PUSHPULL);
//    palSetLineMode(LED_3_LINE,      PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(XL320_OLD_DATA_PIN, PAL_MODE_INPUT);

    palSetLineMode(POWER12_ENABLE_PIN, POWER12_ENABLE_PIN_MODE);
    palSetLineMode(POWER12_IMON_PIN, POWER12_IMON_PIN_MODE);
    palSetLineMode(POWER12_LDSTR_PIN, POWER12_LDSTR_PIN_MODE);
    palSetLineMode(POWER12_PG_PIN, POWER12_PG_PIN_MODE);
    palClearLine(POWER12_LDSTR_PIN);

    palSetLineMode(POWER8_ENABLE_PIN, POWER8_ENABLE_PIN_MODE);
    palSetLineMode(POWER8_IMON_PIN, POWER8_IMON_PIN_MODE);
    palSetLineMode(POWER8_PG_PIN, POWER8_PG_PIN_MODE);
    palSetLineMode(POWER8_LDSTR_PIN, POWER8_LDSTR_PIN_MODE);
    palClearLine(POWER8_LDSTR_PIN);
    chThdSleepMilliseconds(500);

    palSetLineMode(RESISTANCE_MEAS_PIN, RESISTANCE_MEAS_PIN_MODE);
    adcStart(&RESISTANCE_MEAS_DRIVER, &resistanceMeasConf);

    Logging::println("ADC driver state %u", RESISTANCE_MEAS_DRIVER.state);
}

void Board::IO::toggleNucleoLed(){
    palToggleLine(NUCLEO_LED_LINE);
}

void Board::IO::toggleLed2(){
    palToggleLine(LED_2_LINE);
}

void Board::IO::toggleLed3(){
    palToggleLine(LED_3_LINE);
}
uint32_t mean_conv = 0;
void adc_cb(ADCDriver *adcp) {
    mean_conv = 0;
    for(size_t i =0; i < adcp->depth; i++) {
        mean_conv += adcp->samples[i];
    }
    mean_conv /= adcp->depth;
}
static const size_t sampleMaxCnt = 1;
adcsample_t samples[sampleMaxCnt];

constexpr float VDD_VOLTAGE = 3.3;
constexpr float RES1_MAX_VOLTAGE = 2.6;
constexpr float RES1_MIN_VOLTAGE = 2.4;
constexpr float RES2_MAX_VOLTAGE = 0.9;
constexpr float RES2_MIN_VOLTAGE = 0.7;
constexpr float RES3_MAX_VOLTAGE = 0.5;
constexpr float RES3_MIN_VOLTAGE = 0.3;

float Board::IO::getResistanceMeasure() {
    adcConvert(&RESISTANCE_MEAS_DRIVER, &resistanceMeasConvGroup, samples, sampleMaxCnt);

    float voltage = (float)samples[0]* (1. / 4096.) * VDD_VOLTAGE;
//    Logging::println("voltage %.4f", voltage);
    if(voltage < RES1_MAX_VOLTAGE && voltage > RES1_MIN_VOLTAGE) {
        Logging::println("4k7");
    }

    if(voltage < RES2_MAX_VOLTAGE && voltage > RES2_MIN_VOLTAGE) {
        Logging::println("1k");
    }

    if(voltage < RES3_MAX_VOLTAGE && voltage > RES3_MIN_VOLTAGE) {
        Logging::println("470R");
    }
    return 0.;
}
