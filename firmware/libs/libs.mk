
COMMONINC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/Dynamixel2Arduino.cpp
#COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/actuator.h
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/actuator.cpp
COMMONINC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/dxl_c
COMMONCSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/dxl_c/protocol.c
COMMONINC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/utility
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/utility/master.cpp
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/utility/port_handler.cpp
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Dynamixel2Arduino/src/utility/slave.cpp

COMMONINC += $(FIRMWAREDIR)/libs/CanProtocol
COMMONINC += $(FIRMWAREDIR)/libs/CanProtocol/ChibiOS
COMMONCPPSRC += $(FIRMWAREDIR)/libs/CanProtocol/CanProtocol.cpp
COMMONCPPSRC += $(FIRMWAREDIR)/libs/CanProtocol/ChibiOS/CanRxThread.cpp
COMMONCPPSRC += $(FIRMWAREDIR)/libs/CanProtocol/ChibiOS/CanTxThread.cpp

#COMMONINC += $(FIRMWAREDIR)/libs/PCA9635/
#COMMONCPPSRC += $(FIRMWAREDIR)/libs/PCA9635/PCA9635.cpp

COMMONINC += $(FIRMWAREDIR)/libs/Adafruit-PWM-Servo-Driver-Library
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Adafruit-PWM-Servo-Driver-Library/Adafruit_PWMServoDriver.cpp
