
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
COMMONCPPSRC += $(FIRMWAREDIR)/libs/CanProtocol/ChibiOS/CanRxThread.cpp
COMMONCPPSRC += $(FIRMWAREDIR)/libs/CanProtocol/ChibiOS/CanTxThread.cpp

#COMMONINC += $(FIRMWAREDIR)/libs/PCA9635/
#COMMONCPPSRC += $(FIRMWAREDIR)/libs/PCA9635/PCA9635.cpp

COMMONINC += $(FIRMWAREDIR)/libs/Adafruit-PWM-Servo-Driver-Library
COMMONCPPSRC += $(FIRMWAREDIR)/libs/Adafruit-PWM-Servo-Driver-Library/Adafruit_PWMServoDriver.cpp

CYPHALDIR = $(LIBDIR)/CanProtocol/Cyphal
CYPHALLOOKUPDIRS += $(CYPHALDIR)/public_regulated_data_types/uavcan
CYPHALREGULAR += $(CYPHALDIR)/public_regulated_data_types/reg
COMMONINC += $(CYPHALDIR)/libcanard/libcanard/
COMMONCSRC += $(CYPHALDIR)/libcanard/libcanard/canard.c

COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/sensor
COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/common
COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/battery
COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/actuator/servo
COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/actuator/esc
COMMONINC += $(CYPHALDIR)/includes/reg/udral/service/actuator/common/sp
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/time
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/thermodynamics
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/optics
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/kinematics/translation
COMMONINC += $(CYPHALDIR)/includes//reg/udral/physics/kinematics/rotation
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/kinematics/
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/electricity
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/dynamics/translation
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/dynamics/rotation
COMMONINC += $(CYPHALDIR)/includes/reg/udral/physics/acoustics
COMMONINC += $(CYPHALDIR)/includes/uavcan/pnp/cluster
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/angular_acceleration
COMMONINC += $(CYPHALDIR)/includes/uavcan/file
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/frequency
COMMONINC += $(CYPHALDIR)/includes/uavcan/diagnostic
COMMONINC += $(CYPHALDIR)/includes/uavcan/primitive/array
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/force
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/angle
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/duration
COMMONINC += $(CYPHALDIR)/includes/uavcan/node/port
COMMONINC += $(CYPHALDIR)/includes/uavcan/_register
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/electric_current
COMMONINC += $(CYPHALDIR)/includes/uavcan/si
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/angular_velocity
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/length
COMMONINC += $(CYPHALDIR)/includes/uavcan/pnp
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/acceleration
COMMONINC += $(CYPHALDIR)/includes/uavcan/primitive
COMMONINC += $(CYPHALDIR)/includes/uavcan/node
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/energy
COMMONINC += $(CYPHALDIR)/includes/uavcan/metatransport/can
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/electric_charge
COMMONINC += $(CYPHALDIR)/includes/uavcan/primitive/scalar
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/magnetic_field_strength
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/mass
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/power
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/pressure
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/temperature
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/torque
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/velocity
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/voltage
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/volume
COMMONINC += $(CYPHALDIR)/includes/uavcan/si/unit/volumetric_flow_rate
COMMONINC += $(CYPHALDIR)/includes/uavcan/time
COMMONINC += $(CYPHALDIR)/includes/

#$(shell nnvg --target-language c -v --target-endianness=little $(CYPHALREGULAR) --lookup-dir $(CYPHALLOOKUPDIRS) --outdir $(CYPHALDIR)/includes)
#$(shell nnvg --target-language c -v --target-endianness=little $(CYPHALLOOKUPDIRS) --outdir $(CYPHALDIR)/includes)
#To enable asserts use this line instead
#$(shell nnvg --target-language c --target-endianness=little --enable-serialization-asserts $(CYPHALREGULAR) --lookup-dir $(CYPHALLOOKUPDIRS) --outdir $(CYPHALDIR)/includes)
