##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O0 -ggdb -fomit-frame-pointer -falign-functions=16 -specs=nano.specs -specs=nosys.specs -lm
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -std=c++17 -fno-exceptions
endif

# Enable this if you want the linker to remove unused code and data.
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = --print-memory-usage
endif

# Enable this if you want link time optimizations (LTO).
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# FPU-related options.
ifeq ($(USE_FPU_OPT),)
  USE_FPU_OPT = -mfloat-abi=$(USE_FPU) -mfpu=fpv4-sp-d16
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, target, sources and paths
#

# Define project name here
PROJECT = ActionBoard
TARGET_TYPE = hardware
TARGER = robot

# Target settings.
MCU      = cortex-m4
PLATFORM = STM32G4

# Imported source files and paths.
ROOTDIR   := ../../../..
CHIBIOS   := $(ROOTDIR)/firmware/libs/ChibiOS
CONFDIR   := ./cfg
BUILDDIR  := ./build
DEPDIR    := ./.dep
COMMONDIR := $(ROOTDIR)/common
FIRMWAREDIR := $(ROOTDIR)/firmware
SOURCEDIR   := $(FIRMWAREDIR)/src
LIBDIR    := $(FIRMWAREDIR)/libs
PARAMDIR  := ./param

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32g4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32G4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_NUCLEO64_G431RB/board.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMv7-M/compilers/GCC/mk/port.mk
# Auto-build files in ./source recursively.
include $(CHIBIOS)/tools/mk/autobuild.mk
include $(CHIBIOS)/os/hal/lib/streams/streams.mk
include $(CHIBIOS)/os/various/cpp_wrappers/chcpp.mk
include $(CHIBIOS)/os/various/shell/shell.mk

# Define linker script file here
LDSCRIPT= $(STARTUPLD)/STM32G431xB.ld

# Project files
include $(COMMONDIR)/common.mk
include $(FIRMWAREDIR)/firmware.mk

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(COMMONCSRC)

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC) \
         $(COMMONCPPSRC)

# List ASM source files here.
ASMSRC = $(ALLASMSRC)

# List ASM with preprocessor source files here.
ASMXSRC = $(ALLXASMSRC)

# Inclusion directories.
INCDIR = $(CONFDIR) $(ALLINC) $(COMMONINC) $(PARAMDIR) $(SOURCEDIR)

# Define C warning options here.
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here.
CPPWARN = -Wall -Wextra -Wundef

#
# Project, target, sources and paths
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS = -DBLUE_ROBOT
#UDEFS = -DRED_ROBOT

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS =

#
# End of user section
##############################################################################

##############################################################################
# Common rules
#

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/arm-none-eabi.mk
include $(RULESPATH)/rules.mk

#
# Common rules
##############################################################################

##############################################################################
# Custom rules
#

flash:
	make -j
	openocd -s ./cfg/ -c "set BIN_FILE build/${PROJECT}.bin" -f flash.cfg

gdb:
	gnome-terminal --command "openocd -s ./cfg/ -f gdb.cfg"
	gdb-multiarch build/${PROJECT}.elf -x ../gdb_script.gdb
	killall openocd
#
# Custom rules
##############################################################################
