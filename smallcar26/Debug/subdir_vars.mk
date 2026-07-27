################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
SYSCFG_SRCS += \
../gpio_toggle_output.syscfg 

C_SRCS += \
../axis6.c \
../balance.c \
../delay.c \
../encoder.c \
./ti_msp_dl_config.c \
C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c \
../i2c_soft.c \
../imu.c \
../k230_comm.c \
../main.c \
../motor.c \
../oled.c \
../pid.c 

GEN_CMDS += \
./device_linker.cmd 

GEN_FILES += \
./device_linker.cmd \
./device.opt \
./ti_msp_dl_config.c 

C_DEPS += \
./axis6.d \
./balance.d \
./delay.d \
./encoder.d \
./ti_msp_dl_config.d \
./startup_mspm0g350x_ticlang.d \
./i2c_soft.d \
./imu.d \
./k230_comm.d \
./main.d \
./motor.d \
./oled.d \
./pid.d 

GEN_OPTS += \
./device.opt 

OBJS += \
./axis6.o \
./balance.o \
./delay.o \
./encoder.o \
./ti_msp_dl_config.o \
./startup_mspm0g350x_ticlang.o \
./i2c_soft.o \
./imu.o \
./k230_comm.o \
./main.o \
./motor.o \
./oled.o \
./pid.o 

GEN_MISC_FILES += \
./device.cmd.genlibs \
./ti_msp_dl_config.h \
./Event.dot 

OBJS__QUOTED += \
"axis6.o" \
"balance.o" \
"delay.o" \
"encoder.o" \
"ti_msp_dl_config.o" \
"startup_mspm0g350x_ticlang.o" \
"i2c_soft.o" \
"imu.o" \
"k230_comm.o" \
"main.o" \
"motor.o" \
"oled.o" \
"pid.o" 

GEN_MISC_FILES__QUOTED += \
"device.cmd.genlibs" \
"ti_msp_dl_config.h" \
"Event.dot" 

C_DEPS__QUOTED += \
"axis6.d" \
"balance.d" \
"delay.d" \
"encoder.d" \
"ti_msp_dl_config.d" \
"startup_mspm0g350x_ticlang.d" \
"i2c_soft.d" \
"imu.d" \
"k230_comm.d" \
"main.d" \
"motor.d" \
"oled.d" \
"pid.d" 

GEN_FILES__QUOTED += \
"device_linker.cmd" \
"device.opt" \
"ti_msp_dl_config.c" 

C_SRCS__QUOTED += \
"../axis6.c" \
"../balance.c" \
"../delay.c" \
"../encoder.c" \
"./ti_msp_dl_config.c" \
"C:/TI/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c" \
"../i2c_soft.c" \
"../imu.c" \
"../k230_comm.c" \
"../main.c" \
"../motor.c" \
"../oled.c" \
"../pid.c" 

SYSCFG_SRCS__QUOTED += \
"../gpio_toggle_output.syscfg" 


