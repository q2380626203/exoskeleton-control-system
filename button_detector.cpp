#include "button_detector.h"

// 全局变量定义
ButtonData g_buttonData = {0, 0, false, false};

// GPIO1中断服务函数
void IRAM_ATTR button_gpio1_isr() {
    uint32_t currentTime = millis();
    
    // 去抖动处理 - 1秒内只判定一次按下
    if (currentTime - g_buttonData.gpio1_lastInterruptTime > BUTTON_DEBOUNCE_MS) {
        g_buttonData.gpio1_pressed_flag = true;
        g_buttonData.gpio1_lastInterruptTime = currentTime;
    }
}

// GPIO2中断服务函数
void IRAM_ATTR button_gpio2_isr() {
    uint32_t currentTime = millis();
    
    // 去抖动处理 - 1秒内只判定一次按下
    if (currentTime - g_buttonData.gpio2_lastInterruptTime > BUTTON_DEBOUNCE_MS) {
        g_buttonData.gpio2_pressed_flag = true;
        g_buttonData.gpio2_lastInterruptTime = currentTime;
    }
}

void button_init() {
    // 初始化GPIO引脚为输入模式，启用内部上拉电阻
    pinMode(BUTTON_GPIO1_PIN, INPUT_PULLUP);
    pinMode(BUTTON_GPIO2_PIN, INPUT_PULLUP);
    
    // 初始化按键数据结构
    g_buttonData.gpio1_lastInterruptTime = 0;
    g_buttonData.gpio2_lastInterruptTime = 0;
    
    // 绑定中断服务函数，只检测下降沿（按键按下）
    attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO1_PIN), button_gpio1_isr, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO2_PIN), button_gpio2_isr, FALLING);
    
    Serial.println("按键检测模块初始化完成（仅按下检测，1秒去抖）");
    Serial.printf("GPIO1引脚: %d, GPIO2引脚: %d\n", BUTTON_GPIO1_PIN, BUTTON_GPIO2_PIN);
}

// 按键处理任务
void buttonProcessTask(void* pvParameters) {
    while (true) {
        // 检查按键1状态位
        if (g_buttonData.gpio1_pressed_flag) {
            g_buttonData.gpio1_pressed_flag = false;  // 清除状态位
            
            // 按键1：增加爬楼助力值
            assistParams.ASSIST_TORQUE_CLIMB += 1.0f;
            assistParams.ASSIST_TORQUE_CLIMB_STEEP += 1.0f;
            
            Serial.printf("按键1被按下 - 爬楼助力值增加: 普通=%.1f, 大幅=%.1f\n", 
                         assistParams.ASSIST_TORQUE_CLIMB, 
                         assistParams.ASSIST_TORQUE_CLIMB_STEEP);
            
            // 语音播报
            voiceModule.speak("助力增加");
            vTaskDelay(pdMS_TO_TICKS(2000)); // 等待语音播报完成
            voiceModule.speak("当前助力");
            vTaskDelay(pdMS_TO_TICKS(2000)); // 等待语音播报完成
            voiceModule.speakChineseNumber((int)assistParams.ASSIST_TORQUE_CLIMB);
        }
        
        // 检查按键2状态位
        if (g_buttonData.gpio2_pressed_flag) {
            g_buttonData.gpio2_pressed_flag = false;  // 清除状态位
            
            // 按键2：减少爬楼助力值
            assistParams.ASSIST_TORQUE_CLIMB -= 1.0f;
            assistParams.ASSIST_TORQUE_CLIMB_STEEP -= 1.0f;
            
            // 防止负值
            if (assistParams.ASSIST_TORQUE_CLIMB < 0.0f) assistParams.ASSIST_TORQUE_CLIMB = 0.0f;
            if (assistParams.ASSIST_TORQUE_CLIMB_STEEP < 0.0f) assistParams.ASSIST_TORQUE_CLIMB_STEEP = 0.0f;
            
            Serial.printf("按键2被按下 - 爬楼助力值减少: 普通=%.1f, 大幅=%.1f\n", 
                         assistParams.ASSIST_TORQUE_CLIMB, 
                         assistParams.ASSIST_TORQUE_CLIMB_STEEP);
            
            // 语音播报
            voiceModule.speak("助力减少");
            vTaskDelay(pdMS_TO_TICKS(2000)); // 等待语音播报完成
            voiceModule.speak("当前助力");
            vTaskDelay(pdMS_TO_TICKS(2000)); // 等待语音播报完成
            voiceModule.speakChineseNumber((int)assistParams.ASSIST_TORQUE_CLIMB);
        }
        
        // 任务延时，避免过度占用CPU
        vTaskDelay(pdMS_TO_TICKS(50));  // 50ms延时
    }
}