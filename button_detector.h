#ifndef BUTTON_DETECTOR_H
#define BUTTON_DETECTOR_H

#include <Arduino.h>
#include "ble_manager.h"
#include "voice_module.h"

// GPIO引脚配置
#define BUTTON_GPIO1_PIN    1
#define BUTTON_GPIO2_PIN    2

// 按键状态定义
#define BUTTON_RELEASED     HIGH
#define BUTTON_PRESSED      LOW

// 按键去抖动配置
#define BUTTON_DEBOUNCE_MS  1000

// 按键数据结构 - 去抖动和状态位
struct ButtonData {
    volatile uint32_t gpio1_lastInterruptTime;
    volatile uint32_t gpio2_lastInterruptTime;
    volatile bool gpio1_pressed_flag;
    volatile bool gpio2_pressed_flag;
};

// 全局变量声明
extern ButtonData g_buttonData;

// 中断服务函数声明
void IRAM_ATTR button_gpio1_isr();
void IRAM_ATTR button_gpio2_isr();

// 外部变量声明
extern AssistParameters assistParams;
extern VoiceModule voiceModule;

// 函数声明
void button_init();
void buttonProcessTask(void* pvParameters);

#endif // BUTTON_DETECTOR_H