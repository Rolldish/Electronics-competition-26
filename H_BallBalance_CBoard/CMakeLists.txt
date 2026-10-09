cmake_minimum_required(VERSION 3.22)

# STM32CubeMX 生成的底层代码使用 C11，自写控制代码使用 C++20。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(H_BallBalance_CBoard LANGUAGES C ASM CXX)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(
    HBALL_MINIMAL_UART_TEST
    "Run only the USART6 PING/PONG hardware smoke test"
    OFF
)
option(
    HBALL_SIM_VISION_MOTOR_TEST
    "Run the short-arm simulated-vision motor bench test"
    OFF
)
option(
    HBALL_PIPE_ANGLE_TEST
    "Run the installed-mechanism fixed-level angle-loop test"
    OFF
)
option(
    HBALL_PIPE_SWEEP_TEST
    "Run the standalone installed-pipe angle sweep"
    OFF
)
option(
    HBALL_IMU_FEEDFORWARD_TEST
    "Run the standalone BMI088 vehicle-acceleration feedforward test"
    OFF
)

if(HBALL_MINIMAL_UART_TEST AND HBALL_SIM_VISION_MOTOR_TEST)
    message(FATAL_ERROR
        "HBALL_MINIMAL_UART_TEST and HBALL_SIM_VISION_MOTOR_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_MINIMAL_UART_TEST AND HBALL_PIPE_ANGLE_TEST)
    message(FATAL_ERROR
        "HBALL_MINIMAL_UART_TEST and HBALL_PIPE_ANGLE_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_SIM_VISION_MOTOR_TEST AND HBALL_PIPE_ANGLE_TEST)
    message(FATAL_ERROR
        "HBALL_SIM_VISION_MOTOR_TEST and HBALL_PIPE_ANGLE_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_MINIMAL_UART_TEST AND HBALL_PIPE_SWEEP_TEST)
    message(FATAL_ERROR
        "HBALL_MINIMAL_UART_TEST and HBALL_PIPE_SWEEP_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_SIM_VISION_MOTOR_TEST AND HBALL_PIPE_SWEEP_TEST)
    message(FATAL_ERROR
        "HBALL_SIM_VISION_MOTOR_TEST and HBALL_PIPE_SWEEP_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_PIPE_ANGLE_TEST AND HBALL_PIPE_SWEEP_TEST)
    message(FATAL_ERROR
        "HBALL_PIPE_ANGLE_TEST and HBALL_PIPE_SWEEP_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_MINIMAL_UART_TEST AND HBALL_IMU_FEEDFORWARD_TEST)
    message(FATAL_ERROR
        "HBALL_MINIMAL_UART_TEST and HBALL_IMU_FEEDFORWARD_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_SIM_VISION_MOTOR_TEST AND HBALL_IMU_FEEDFORWARD_TEST)
    message(FATAL_ERROR
        "HBALL_SIM_VISION_MOTOR_TEST and HBALL_IMU_FEEDFORWARD_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_PIPE_ANGLE_TEST AND HBALL_IMU_FEEDFORWARD_TEST)
    message(FATAL_ERROR
        "HBALL_PIPE_ANGLE_TEST and HBALL_IMU_FEEDFORWARD_TEST "
        "cannot be enabled together"
    )
endif()
if(HBALL_PIPE_SWEEP_TEST AND HBALL_IMU_FEEDFORWARD_TEST)
    message(FATAL_ERROR
        "HBALL_PIPE_SWEEP_TEST and HBALL_IMU_FEEDFORWARD_TEST "
        "cannot be enabled together"
    )
endif()

if(HBALL_MINIMAL_UART_TEST)
    set(HBALL_APPLICATION_SOURCES
        Application/MinimalUartProtocol.c
        Application/MinimalUartTest.c
    )
else()
    # 这里列出本项目自己维护的业务代码；新增 .cpp 文件后通常要在这里登记。
    set(HBALL_APPLICATION_SOURCES
        Application/BallBalanceController.cpp
        Application/BallControlTask.cpp
        Application/BoardIo.cpp
        Application/CompetitionImuFeedforward.cpp
        Application/CompetitionTaskGuard.cpp
        Application/BallLinkProtocol.cpp
        Application/BallLinkReceiver.cpp
        Application/BallLinkStreamParser.cpp
        Application/ImuAccelTask.cpp
        Application/ImuFeedforwardController.cpp
        Application/ImuVehicleSnapshotStore.cpp
        Application/ImuRuntimeMonitor.cpp
        Application/VehicleAccelEstimator.cpp
        Application/VehicleLaunchFeedforward.cpp
        Application/VehicleLaunchTrace.cpp
        Application/VisionCommTask.cpp
        Libraries/BMI088/BMI088.cpp
        Libraries/QD4310/QD4310.cpp
    )
endif()

add_executable(
    H_BallBalance_CBoard
    ${HBALL_APPLICATION_SOURCES}
    Application/SafetyWatchdog.c
)

# 加入 CubeMX 生成的启动、HAL、FreeRTOS 和外设初始化代码。
add_subdirectory(cmake/stm32cubemx)

# 让业务代码可以直接 #include "BoardIo.h" 和 "QD4310.h"。
target_include_directories(H_BallBalance_CBoard PRIVATE
    Application
    Libraries/BMI088
    Libraries/QD4310
)

target_link_libraries(H_BallBalance_CBoard
    stm32cubemx
)

if(HBALL_MINIMAL_UART_TEST)
    target_compile_definitions(
        H_BallBalance_CBoard
        PRIVATE HBALL_MINIMAL_UART_TEST=1
    )
endif()

if(HBALL_SIM_VISION_MOTOR_TEST)
    target_compile_definitions(
        H_BallBalance_CBoard
        PRIVATE HBALL_SIM_VISION_MOTOR_TEST=1
    )
endif()

if(HBALL_PIPE_ANGLE_TEST)
    target_compile_definitions(
        H_BallBalance_CBoard
        PRIVATE HBALL_PIPE_ANGLE_TEST=1
    )
endif()

if(HBALL_PIPE_SWEEP_TEST)
    target_compile_definitions(
        H_BallBalance_CBoard
        PRIVATE HBALL_PIPE_SWEEP_TEST=1
    )
endif()

if(HBALL_IMU_FEEDFORWARD_TEST)
    target_compile_definitions(
        H_BallBalance_CBoard
        PRIVATE HBALL_IMU_FEEDFORWARD_TEST=1
    )
endif()

# 编译结束后显示 Flash/RAM 占用。
add_custom_command(TARGET H_BallBalance_CBoard POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE_NAME:H_BallBalance_CBoard>
    WORKING_DIRECTORY $<TARGET_FILE_DIR:H_BallBalance_CBoard>
)
