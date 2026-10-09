#pragma once

#include <stdint.h>

/*
 * C 板按键和 RGB 指示灯的统一接口。
 *
 * 上层控制代码只需要关心“按键是否按下”和“当前处于什么状态”，
 * 不需要知道具体 GPIO 端口、引脚和电平高低。
 */
enum class BoardLedStatus : uint8_t {
    WaitingForFeedback, // 蓝灯常亮：还没收到可靠反馈，禁止启动。
    Ready,              // 绿灯常亮：电机安全，可以按 KEY 启动。
    Running,            // 绿灯闪烁：电机正在运行。
    Fault               // 红灯常亮：发生故障，等待安全恢复。
};

// 每个 5 ms 控制周期调用一次，用连续采样完成按键消抖。
// 这两个按键函数只允许在 BallControlTask 中使用，不要在中断里调用。
void BoardIo_UpdateKey();

// 返回消抖后的按键状态：true 表示 KEY 正在被按下。
bool BoardIo_KeyPressed();

// 根据系统状态刷新 RGB 灯；nowMs 用来计算运行状态的闪烁节拍。
void BoardIo_SetLed(BoardLedStatus status, uint32_t nowMs);
