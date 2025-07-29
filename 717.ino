/*
 * =================================================================================================
 * 项目名称：登山外骨骼 (Exoskeleton for Mountaineering)
 * 版本：V3 - 纯蓝牙控制版
 * 核心控制：ESP32
 * 主要功能：
 * 1. 通过BLE（低功耗蓝牙）与Web蓝牙应用通信。
 * 2. 实时传输电机状态数据到Web应用。
 * 3. 通过Web应用实时动态调整所有助力参数。
 * 4. 使用FreeRTOS多任务处理，保证系统实时性和稳定性。
 * 5. 通过UART总线与两台小米CyberGear微电机进行同步轮询通信。
 * 6. 实时分析用户运动姿态，可区分"静止"、"平地行走"、"爬楼"和"大幅度爬楼"。
 * 7. 在"自适应模式"下，根据分析结果和动态参数，提供平滑的辅助力矩。
 * =================================================================================================
 */

// ==================== 必要的头文件 ===================
#include <HardwareSerial.h>
#include "rs01_motor.h"
#include "gy25t_sensor.h"
#include "ble_manager.h"
#include "gps_module.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define ACTIVITY_BUFFER_SIZE 50

// ==================== 助力模式参数结构体 (支持动态修改) ===================
// AssistParameters结构体已在ble_manager.h中定义

AssistParameters assistParams = {
    0.15f, 0.6f, 1.0f, 4.0f, 7.0f, 8.0f, 0.2f, true, 0.5f, 0.1f, false
};

// ==================== 核心数据结构与状态机 ===================
// 枚举和ActivityDataPoint已在ble_manager.h中定义
struct MotorChannel {
    MI_Motor motor_obj;
    volatile SystemState systemState = STATE_STANDBY;
    volatile DetectedActivity detectedActivity = ACTIVITY_UNKNOWN;
    volatile DetectedActivity lastActivity = ACTIVITY_UNKNOWN;
    volatile float target_torque = 0.0f;
    ActivityDataPoint activityBuffer[ACTIVITY_BUFFER_SIZE];
    volatile int activityBufferIndex = 0;
    volatile UartCommandType pending_web_cmd = CMD_NONE;
    volatile float pending_web_value = 0.0f;
    volatile bool activityChanged = false;
    volatile uint32_t activityChangeTime = 0;
    volatile bool waitingForEnableDelay = false;
    volatile uint16_t standstillCounter = 0;
    volatile bool standstillConfirmed = false;
    volatile uint32_t intermittentCycleStartTime = 0;
    volatile bool isHighTorquePeriod = true;
};

// ==================== 全局变量与任务句柄 ===================
MotorChannel motorChannels[2];
SemaphoreHandle_t motor1DataReceivedSemaphore;
SemaphoreHandle_t motor2DataReceivedSemaphore;
TaskHandle_t uartReceiveParseTaskHandle = NULL;
TaskHandle_t uartTransmitManagerTaskHandle = NULL;
TaskHandle_t analysisTaskHandle = NULL;
TaskHandle_t standstillDetectionTaskHandle = NULL;
TaskHandle_t gyroUpdateTaskHandle = NULL;
TaskHandle_t gpsUpdateTaskHandle = NULL;

// BLE管理器实例在ble_manager.cpp中定义

// 初始化完成标志，确保模式设置完成后才开始发送电流指令
volatile bool motorInitializationComplete = false;

// 多阶段检测状态枚举
enum DetectionPhase {
    PHASE_NORMAL = 0,        // 正常运行：电机位置主检测
    PHASE_FORCE_STANDSTILL,  // 强制静止：2秒静止期 + 力矩衰减
    PHASE_RECOVERY_TRANSITION, // 恢复过渡：纯陀螺仪检测期
    PHASE_FULL_RECOVERY      // 完全恢复：正常双重检测
};

// 静止检测控制变量
volatile uint8_t standstillDetectionCount[2] = {1, 1}; // 静止检测次数标记，默认为1
volatile DetectionPhase detectionPhase[2] = {PHASE_NORMAL, PHASE_NORMAL}; // 检测阶段
volatile uint32_t phaseStartTime[2] = {0, 0};          // 阶段开始时间
volatile bool forceStandstillState[2] = {false, false}; // 强制静止状态标志
volatile uint32_t forceStandstillEndTime[2] = {0, 0};   // 强制静止状态结束时间
volatile bool torqueDecayActive[2] = {false, false};    // 力矩衰减激活标志（强制静止检测使用）
volatile float torqueDecayStartValue[2] = {0.0f, 0.0f}; // 衰减开始时的力矩值
volatile uint32_t torqueDecayStartTime[2] = {0, 0};     // 衰减开始时间

// 状态机静止检测的力矩衰减控制变量
volatile bool stateMachineTorqueDecayActive[2] = {false, false};    // 状态机力矩衰减激活标志
volatile float stateMachineTorqueDecayStartValue[2] = {0.0f, 0.0f}; // 状态机衰减开始时的力矩值
volatile uint32_t stateMachineTorqueDecayStartTime[2] = {0, 0};     // 状态机衰减开始时间

// ==================== 函数声明 ===================
void uartReceiveParseTask(void* parameter);
void uartTransmitManagerTask(void* parameter);
void analysisTask(void* parameter);
void standstillDetectionTask(void* parameter);
void gyroUpdateTask(void* parameter);
void gpsUpdateTask(void* parameter);
void analyzeActivityAndSetTorque(MotorChannel& channel);
void motorDataCallback(MI_Motor* updated_motor);
float processStepwiseTorqueDecay(int motor_index, bool isActive, float startValue, uint32_t startTime, const char* source);
float getMotorPositionRange(MotorChannel& channel, int sampleCount = ACTIVITY_BUFFER_SIZE);
bool gy25t_isStandstill();

// ==================== BLE接口函数实现 ===================
void setMotorSystemState(int motor_index, SystemState state) {
    if (motor_index >= 0 && motor_index < 2) {
        motorChannels[motor_index].systemState = state;
    }
}

void setMotorPendingCommand(int motor_index, UartCommandType cmd, float value) {
    if (motor_index >= 0 && motor_index < 2) {
        motorChannels[motor_index].pending_web_cmd = cmd;
        motorChannels[motor_index].pending_web_value = value;
    }  
}

SystemState getMotorSystemState(int motor_index) {
    if (motor_index >= 0 && motor_index < 2) {
        return motorChannels[motor_index].systemState;
    }
    return STATE_STANDBY;
}

void setMotorTargetTorque(int motor_index, float torque) {
    if (motor_index >= 0 && motor_index < 2) {
        motorChannels[motor_index].target_torque = torque;
    }
}

// ==================== Arduino Setup ===================
void setup() {
    Serial.begin(115200);
    
    // 上电后等待3秒，让电机驱动器和其他系统稳定启动
    Serial.println("系统上电，等待3秒让所有子系统稳定...");
    for (int i = 3; i > 0; i--) {
        Serial.printf("等待倒计时: %d秒\n", i);
        delay(1000);
    }
    Serial.println("系统稳定，开始初始化...");
    
    motorChannels[0].motor_obj.id = MOTER_1_ID;
    motorChannels[1].motor_obj.id = MOTER_2_ID;
    
    for (int i = 0; i < 2; i++) {
        motorChannels[i].activityChanged = false;
        motorChannels[i].waitingForEnableDelay = false;
        motorChannels[i].lastActivity = ACTIVITY_UNKNOWN;
        motorChannels[i].standstillCounter = 0;
        motorChannels[i].standstillConfirmed = false;
        motorChannels[i].intermittentCycleStartTime = 0;
        motorChannels[i].isHighTorquePeriod = true;
        
        // 初始化多阶段检测系统
        detectionPhase[i] = PHASE_NORMAL;
        phaseStartTime[i] = 0;
    }

    motor1DataReceivedSemaphore = xSemaphoreCreateBinary();
    motor2DataReceivedSemaphore = xSemaphoreCreateBinary();
    UART_Rx_Init(motorDataCallback);
    
    // 初始化GY25T陀螺仪
    gy25t_init();
    
    // 初始化GPS模块
    gps_init();

    // 首先创建UART和分析任务（提高电机通信任务优先级）
    xTaskCreatePinnedToCore(uartReceiveParseTask, "UartReceiveTask", 8192, NULL, 7, &uartReceiveParseTaskHandle, 1);  // 提高到7
    xTaskCreatePinnedToCore(uartTransmitManagerTask, "UartTransmitTask", 8192, NULL, 6, &uartTransmitManagerTaskHandle, 1);  // 提高到6
    xTaskCreatePinnedToCore(gyroUpdateTask, "GyroUpdateTask", 4096, NULL, 4, &gyroUpdateTaskHandle, 0);  // 提高到4
    xTaskCreatePinnedToCore(gpsUpdateTask, "GpsUpdateTask", 4096, NULL, 3, &gpsUpdateTaskHandle, 0);  // GPS任务优先级3
    xTaskCreatePinnedToCore(analysisTask, "AnalysisTask", 8192, NULL, 2, &analysisTaskHandle, 0);  // 降低到2
    xTaskCreatePinnedToCore(standstillDetectionTask, "StandstillDetectionTask", 4096, NULL, 1, &standstillDetectionTaskHandle, 0);  // 降低到1
    Serial.println("所有FreeRTOS任务已创建.");
    
    // 初始化电机模式为电流模式
    Serial.println("正在设置电机工作模式为电流模式...");
    delay(500); // 给UART任务一些时间启动
    
    for (int i = 0; i < 2; i++) {
        Change_Mode(&motorChannels[i].motor_obj, CUR_MODE);
        delay(100); // 每个电机设置后稍作延时
        Serial.printf("电机 %d 已设置为电流模式\n", motorChannels[i].motor_obj.id);
    }
    Serial.println("所有电机已设置为电流模式.");
    
    // 设置初始化完成标志，允许uartTransmitManagerTask开始发送电流指令
    motorInitializationComplete = true;
    Serial.println("电机初始化完成，现在可以安全发送电流指令.");
    
    // 初始化BLE管理器
    bleManager.init();
}

// 在loop函数中添加BLE管理
void loop() {
    // 添加BLE连接监控（不自动重启）
    static unsigned long lastForceSendTime = 0;
    
    if (bleManager.isConnected()) {
        // 每5秒主动发送一次所有数据，确保app能接收到
        if (millis() - lastForceSendTime > 5000) {
            bleManager.sendAllData();
            lastForceSendTime = millis();
        }
        
        // 定期发送电机数据
        static unsigned long lastNotifyTime = 0;
        if (millis() - lastNotifyTime > 500) {
            bleManager.setShouldSendMotorData(true);
            lastNotifyTime = millis();
        }
    }
    
    // 处理BLE管理器的循环逻辑
    bleManager.handleLoop();
    
    vTaskDelay(pdMS_TO_TICKS(10));
}

// ==================== FreeRTOS 任务实现 ===================
void analysisTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(50);
    const uint16_t STANDSTILL_THRESHOLD = 40;

    for (;;) {
        for (int i = 0; i < 2; i++) {
            MotorChannel& channel = motorChannels[i];
            if (channel.systemState == STATE_AUTO_ADAPT) {
                analyzeActivityAndSetTorque(channel);
                if (channel.detectedActivity == ACTIVITY_STANDSTILL) {
                    if (channel.standstillCounter < 65535) channel.standstillCounter++;
                    if (channel.standstillCounter >= STANDSTILL_THRESHOLD && !channel.standstillConfirmed && assistParams.AUTO_DISABLE_ENABLED) {
                        channel.standstillConfirmed = true;
                        Motor_Reset(&channel.motor_obj, 0); // The new Motor_Reset function requires a second argument.
                        Serial.printf("电机 %d: 检测到持续静止2秒，已直接失能\n", channel.motor_obj.id);
                    }
                } else {
                    channel.standstillCounter = 0;
                    channel.standstillConfirmed = false;
                }
            } else {
                channel.target_torque = (assistParams.TORQUE_SMOOTHING_FACTOR * 0.0f) + ((1.0f - assistParams.TORQUE_SMOOTHING_FACTOR) * channel.target_torque);
                if (abs(channel.target_torque) < 0.01) channel.target_torque = 0.0f;
                channel.detectedActivity = ACTIVITY_STANDSTILL;
                channel.standstillCounter = 0;
                channel.standstillConfirmed = false;
            }
        }
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

void uartTransmitManagerTask(void* parameter) {
    uint8_t next_motor_to_poll = MOTER_1_ID;
    const uint32_t INTERMITTENT_CYCLE_MS = 50;
    
    // 超时打印控制变量
    static unsigned long lastTimeoutPrintTime[2] = {0, 0}; // 分别为电机1和电机2
    static const unsigned long TIMEOUT_PRINT_INTERVAL = 5000; // 5秒间隔
    
    // 等待电机初始化完成
    while (!motorInitializationComplete) {
        Serial.println("等待电机模式初始化完成...");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    Serial.println("电机初始化完成，uartTransmitManagerTask开始正常工作");
    
    for (;;) {
        MotorChannel& channel = motorChannels[next_motor_to_poll - 1];
        MI_Motor* motor_to_poll = &channel.motor_obj;
        SemaphoreHandle_t semaphore_to_wait_for = (next_motor_to_poll == MOTER_1_ID) ? motor1DataReceivedSemaphore : motor2DataReceivedSemaphore;
        xSemaphoreTake(semaphore_to_wait_for, (TickType_t)0);
        
        // 1. 处理一次性指令 (来自Web/蓝牙)
        if (channel.pending_web_cmd != CMD_NONE) {
            Serial.printf("uartTransmitManagerTask: 电机%d执行命令 cmd=%d, value=%.0f\n", 
                         next_motor_to_poll, (int)channel.pending_web_cmd, channel.pending_web_value);
            switch(channel.pending_web_cmd) {
                case CMD_ENABLE_MOTOR: 
                    Motor_Enable(motor_to_poll); 
                    Serial.printf("电机%d已使能\n", next_motor_to_poll);
                    break;
                case CMD_DISABLE_MOTOR: 
                    Motor_Reset(motor_to_poll, 0); // The new Motor_Reset function requires a second argument.
                    Serial.printf("电机%d已禁用\n", next_motor_to_poll);
                    break;
                case CMD_CHANGE_MODE: 
                    Change_Mode(motor_to_poll, (uint8_t)channel.pending_web_value);
                    Serial.printf("电机%d已切换到模式%d\n", next_motor_to_poll, (int)channel.pending_web_value);
                    break;
                default: 
                    break;
            }
            channel.pending_web_cmd = CMD_NONE;
            vTaskDelay(pdMS_TO_TICKS(5)); // 发送特殊指令后短暂延时
        }
        
        // 2. 处理基于活动状态的自动使能
        if (channel.systemState == STATE_AUTO_ADAPT && channel.activityChanged) {
            if (channel.lastActivity == ACTIVITY_STANDSTILL && channel.detectedActivity != ACTIVITY_STANDSTILL) {
                Motor_Enable(motor_to_poll);
                Serial.printf("电机 %d: 检测到活动，已使能\n", next_motor_to_poll);
                channel.waitingForEnableDelay = true;
                channel.activityChangeTime = millis();
            }
            channel.activityChanged = false; // 总是清除标志
        }
        
        // 3. 核心：总是发送力矩指令来轮询数据
        float applied_torque = 0.0f; // 默认力矩为0

        // 检查是否在使能后的延迟期
        if (channel.waitingForEnableDelay) {
            if (millis() - channel.activityChangeTime >= 100) {
                channel.waitingForEnableDelay = false; // 延迟结束
            } else {
                // 在延迟期间，继续发送0力矩指令以保持轮询
                applied_torque = 0.0f;
            }
        }
        
        // 如果不在延迟期，则应用计算出的力矩
        if (!channel.waitingForEnableDelay) {
             if (channel.systemState == STATE_EMERGENCY_STOP) {
                applied_torque = 0.0f;
            } else {
                applied_torque = channel.target_torque;

                // 处理间歇式力矩输出逻辑
                if (assistParams.INTERMITTENT_ENABLED && applied_torque != 0) {
                    uint32_t current_time = millis();
                    if (current_time - channel.intermittentCycleStartTime >= INTERMITTENT_CYCLE_MS) {
                        channel.intermittentCycleStartTime = current_time;
                        channel.isHighTorquePeriod = true;
                    }
                    if (channel.isHighTorquePeriod && (current_time - channel.intermittentCycleStartTime) >= (INTERMITTENT_CYCLE_MS * assistParams.INTERMITTENT_DUTY)) {
                        channel.isHighTorquePeriod = false;
                    }
                    if (!channel.isHighTorquePeriod) {
                        applied_torque *= (1.0f - assistParams.INTERMITTENT_REDUCTION);
                    }
                }
            }
        }

        // 发送最终计算出的力矩指令，这是获取电机数据的关键
        Set_CurMode(motor_to_poll, applied_torque);

        if (xSemaphoreTake(semaphore_to_wait_for, pdMS_TO_TICKS(300)) != pdTRUE) {  // 增加超时时间到300ms
            // 控制超时打印频率为5秒一次
            unsigned long currentTime = millis();
            int motorIndex = (next_motor_to_poll == MOTER_1_ID) ? 0 : 1;
            
            if (currentTime - lastTimeoutPrintTime[motorIndex] >= TIMEOUT_PRINT_INTERVAL) {
                Serial.printf("电机 %d 响应超时.\n", next_motor_to_poll);
                lastTimeoutPrintTime[motorIndex] = currentTime;
            }
        }
        next_motor_to_poll = (next_motor_to_poll == MOTER_1_ID) ? MOTER_2_ID : MOTER_1_ID;
        
        // 添加小的延时，避免过度占用CPU
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void standstillDetectionTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 每0.1秒检查一次，提高响应速度
    
    for (;;) {
        uint32_t currentTime = millis();
        
        // 先检查电机位置静止状态
        bool motorPositionStandstill[2] = {false, false};
        for (int i = 0; i < 2; i++) {
            MotorChannel& channel = motorChannels[i];
            float pos_range_500ms = getMotorPositionRange(channel, 10);
            motorPositionStandstill[i] = (pos_range_500ms < 0.20f);
        }
        
        // 再检查陀螺仪静止状态（使用滑动窗口）
        bool gyroStandstill = gy25t_isStandstill();
        bool gyroHasMovement = false;
        if (g_gyroData.dataValid) {
            int16_t abs_x = abs(g_gyroData.x);
            int16_t abs_y = abs(g_gyroData.y);
            int16_t abs_z = abs(g_gyroData.z);
            gyroHasMovement = (abs_x >= 100) || (abs_y >= 100) || (abs_z >= 100);
        }
        
        // 为每个电机处理多阶段检测逻辑
        for (int i = 0; i < 2; i++) {
            MotorChannel& channel = motorChannels[i];
            
            switch (detectionPhase[i]) {
                case PHASE_NORMAL:
                    // 阶段1：正常运行 - 电机位置主检测
                    if (standstillDetectionCount[i] == 0) {
                        bool shouldTriggerStandstill = false;
                        
                        if (!g_gyroData.dataValid) {
                            // 陀螺仪无效，仅使用位置检测
                            shouldTriggerStandstill = motorPositionStandstill[i];
                        } else {
                            // 陀螺仪有效，使用双重检测
                            shouldTriggerStandstill = motorPositionStandstill[i] && gyroStandstill;
                        }
                        
                        if (shouldTriggerStandstill) {
                            // 触发强制静止状态
                            detectionPhase[i] = PHASE_FORCE_STANDSTILL;
                            phaseStartTime[i] = currentTime;
                            forceStandstillState[i] = true;
                            forceStandstillEndTime[i] = currentTime + 2000; // 2秒强制静止
                            channel.detectedActivity = ACTIVITY_STANDSTILL;
                            
                            // 启动力矩衰减
                            torqueDecayActive[i] = true;
                            torqueDecayStartValue[i] = channel.target_torque;
                            torqueDecayStartTime[i] = currentTime;
                            
                            Serial.printf("电机%d 进入PHASE_FORCE_STANDSTILL，开始2秒强制静止+力矩衰减\n", channel.motor_obj.id);
                        }
                    }
                    break;
                    
                case PHASE_FORCE_STANDSTILL:
                    // 阶段2：强制静止期 - 2秒内进行力矩衰减，期间清空位置缓冲区
                    if (currentTime >= forceStandstillEndTime[i]) {
                        // 2秒强制静止期结束，进入恢复过渡期
                        detectionPhase[i] = PHASE_RECOVERY_TRANSITION;
                        phaseStartTime[i] = currentTime;
                        forceStandstillState[i] = false;
                        torqueDecayActive[i] = false;
                        
                        Serial.printf("电机%d 进入PHASE_RECOVERY_TRANSITION，开始纯陀螺仪检测期\n", channel.motor_obj.id);
                    } else {
                        // 仍在强制静止期内，定期清空位置缓冲区避免惯性污染
                        if ((currentTime - phaseStartTime[i]) % 200 == 0) { // 每200ms清空一次
                            // 将缓冲区的位置数据设为当前位置，消除惯性影响
                            float currentPos = channel.motor_obj.position;
                            for (int j = 0; j < ACTIVITY_BUFFER_SIZE; j++) {
                                channel.activityBuffer[j].position = currentPos;
                            }
                        }
                    }
                    break;
                    
                case PHASE_RECOVERY_TRANSITION:
                    // 阶段3：恢复过渡期 - 仅使用陀螺仪检测运动，避免被污染的位置数据影响
                    if (g_gyroData.dataValid && gyroHasMovement) {
                        // 陀螺仪检测到运动，进入完全恢复期
                        detectionPhase[i] = PHASE_FULL_RECOVERY;
                        phaseStartTime[i] = currentTime;
                        standstillDetectionCount[i] = 1; // 恢复状态机
                        
                        Serial.printf("电机%d 陀螺仪检测到运动(X:%d, Y:%d, Z:%d)，进入PHASE_FULL_RECOVERY\n", 
                                     channel.motor_obj.id, g_gyroData.x, g_gyroData.y, g_gyroData.z);
                    } else if (!g_gyroData.dataValid) {
                        // 如果陀螺仪数据无效，等待一段时间后直接恢复
                        if (currentTime - phaseStartTime[i] >= 1000) { // 等待1秒
                            detectionPhase[i] = PHASE_FULL_RECOVERY;
                            phaseStartTime[i] = currentTime;
                            standstillDetectionCount[i] = 1;
                            
                            Serial.printf("电机%d 陀螺仪无效，等待1秒后进入PHASE_FULL_RECOVERY\n", channel.motor_obj.id);
                        }
                    }
                    break;
                    
                case PHASE_FULL_RECOVERY:
                    // 阶段4：完全恢复 - 恢复正常的双重检测逻辑
                    if (currentTime - phaseStartTime[i] >= 500) { // 完全恢复期持续0.5秒
                        detectionPhase[i] = PHASE_NORMAL;
                        Serial.printf("电机%d 完全恢复到PHASE_NORMAL\n", channel.motor_obj.id);
                    }
                    break;
            }
        }
        
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

void uartReceiveParseTask(void* parameter) {
    for (;;) {
        handle_uart_rx();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void analyzeActivityAndSetTorque(MotorChannel& channel) {
    int motor_index = channel.motor_obj.id - 1;
    
    // 检查多阶段检测状态
    if (detectionPhase[motor_index] != PHASE_NORMAL) {
        switch (detectionPhase[motor_index]) {
            case PHASE_FORCE_STANDSTILL:
                // 强制静止期：处理力矩衰减
                channel.detectedActivity = ACTIVITY_STANDSTILL;
                if (torqueDecayActive[motor_index]) {
                    channel.target_torque = processStepwiseTorqueDecay(motor_index, torqueDecayActive[motor_index], 
                                                                      torqueDecayStartValue[motor_index], 
                                                                      torqueDecayStartTime[motor_index], "强制静止");
                    if (channel.target_torque == 0.0f) {
                        torqueDecayActive[motor_index] = false;
                    }
                } else {
                    channel.target_torque = 0.0f;
                }
                return;
                
            case PHASE_RECOVERY_TRANSITION:
            case PHASE_FULL_RECOVERY:
                // 恢复过渡期和完全恢复期：保持静止状态，等待阶段结束
                channel.target_torque = 0.0f;
                channel.detectedActivity = ACTIVITY_STANDSTILL;
                return;
        }
    }
    
    float raw_target_torque = 0.0f;
    DetectedActivity previousActivity = channel.detectedActivity;
    
    // 使用统一的位置检测方法：获取位置变化范围
    float pos_range = getMotorPositionRange(channel, 40); 

    // 基于位置变化范围判断运动状态
    if (pos_range > assistParams.MOVEMENT_THRESHOLD_CLIMB_STEEP) {
        channel.detectedActivity = ACTIVITY_CLIMBING_STEEP;
        raw_target_torque = (channel.motor_obj.id == MOTER_1_ID) ? -assistParams.ASSIST_TORQUE_CLIMB_STEEP : assistParams.ASSIST_TORQUE_CLIMB_STEEP;
    } else if (pos_range > assistParams.MOVEMENT_THRESHOLD_CLIMB) {
        channel.detectedActivity = ACTIVITY_CLIMBING;
        raw_target_torque = (channel.motor_obj.id == MOTER_1_ID) ? -assistParams.ASSIST_TORQUE_CLIMB : assistParams.ASSIST_TORQUE_CLIMB;
    } else if (pos_range > assistParams.MOVEMENT_THRESHOLD_WALK) {
        channel.detectedActivity = ACTIVITY_WALKING;
        raw_target_torque = (channel.motor_obj.id == MOTER_1_ID) ? -assistParams.ASSIST_TORQUE_WALK : assistParams.ASSIST_TORQUE_WALK;
    } else {
        channel.detectedActivity = ACTIVITY_STANDSTILL;
        
        // 检查是否需要启动阶梯式力矩衰减
        if (!stateMachineTorqueDecayActive[motor_index] && abs(channel.target_torque) > 0.1f) {
            // 首次检测到静止且当前有力矩输出，启动阶梯式衰减
            stateMachineTorqueDecayActive[motor_index] = true;
            stateMachineTorqueDecayStartValue[motor_index] = channel.target_torque;
            stateMachineTorqueDecayStartTime[motor_index] = millis();
            Serial.printf("电机%d 状态机检测到静止，启动阶梯式力矩衰减(从%.2f到0，每0.3秒降低2Nm)\n", 
                         channel.motor_obj.id, channel.target_torque);
        }
        
        // 处理阶梯式力矩衰减
        if (stateMachineTorqueDecayActive[motor_index]) {
            raw_target_torque = processStepwiseTorqueDecay(motor_index, stateMachineTorqueDecayActive[motor_index], 
                                                          stateMachineTorqueDecayStartValue[motor_index], 
                                                          stateMachineTorqueDecayStartTime[motor_index], "状态机");
            
            // 检查衰减是否完成
            if (raw_target_torque == 0.0f) {
                stateMachineTorqueDecayActive[motor_index] = false;
            }
        } else {
            raw_target_torque = 0.0f;
        }
    }

    if (previousActivity != channel.detectedActivity) {
        if (previousActivity == ACTIVITY_STANDSTILL && channel.detectedActivity != ACTIVITY_STANDSTILL) {
            // 检测到从静止到运动的状态变化，将标记设为0
            standstillDetectionCount[motor_index] = 0;
            channel.activityChanged = true;
            channel.activityChangeTime = millis();
            channel.lastActivity = previousActivity;
            
            // 重置状态机的力矩衰减标志
            stateMachineTorqueDecayActive[motor_index] = false;
            
            Serial.printf("电机%d 检测到运动变化，标记设为0，重置状态机衰减\n", channel.motor_obj.id);
        }
        
        // 直接设置目标力矩
        channel.target_torque = raw_target_torque;
    } else {
        // 即使活动状态没变，也检查力矩是否需要调整（如参数变化）
        float currentTargetTorque = channel.target_torque;
        if (abs(raw_target_torque - currentTargetTorque) >= 0.1f) {
            channel.target_torque = raw_target_torque;
        }
    }
    if (abs(channel.target_torque) < 0.01) {
        channel.target_torque = 0.0f;
    }
}

// ==================== GY25T陀螺仪更新任务 ===================
void gyroUpdateTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(100); // 100ms
    
    unsigned long lastPrintTime = 0;
    const unsigned long printInterval = 1000; // 每秒打印一次数据
    
    for (;;) {
        // 更新陀螺仪数据
        gy25t_update();
        
        // 每秒打印一次陀螺仪数据用于调试
        // unsigned long currentTime = millis();
        // if (currentTime - lastPrintTime >= printInterval) {
        //     if (g_gyroData.dataValid) {
        //         Serial.printf("陀螺仪数据 - X:%d, Y:%d, Z:%d (时间:%lu)\n", 
        //                       g_gyroData.x, g_gyroData.y, g_gyroData.z, g_gyroData.lastUpdateTime);
        //     } else {
        //         Serial.println("陀螺仪数据无效或未收到数据");
        //     }
        //     lastPrintTime = currentTime;
        // }
        
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

// ==================== 阶梯式力矩衰减函数 ===================
/**
 * 处理阶梯式力矩衰减
 * @param motor_index 电机索引 (0或1)
 * @param isActive 衰减是否激活
 * @param startValue 衰减开始时的力矩值
 * @param startTime 衰减开始时间
 * @param source 调用来源（用于日志标识）
 * @return 当前应该输出的力矩值
 */
float processStepwiseTorqueDecay(int motor_index, bool isActive, float startValue, uint32_t startTime, const char* source) {
    if (!isActive) {
        return 0.0f;
    }
    
    uint32_t elapsedTime = millis() - startTime;
    const uint32_t STEP_INTERVAL = 500; // 每个阶梯
    const float STEP_SIZE = 3.0f; // 每个阶梯
    
    // 计算当前应该在第几个阶梯（从第1阶梯开始就减力）
    uint32_t stepCount = elapsedTime / STEP_INTERVAL;
    float initialTorque = abs(startValue); // 取绝对值计算
    
    // 计算当前阶梯的目标力矩（第1阶梯就开始衰减）
    float currentStepTorque = initialTorque - ((stepCount + 1) * STEP_SIZE);
    
    if (currentStepTorque <= 0.0f) {
        // 衰减完成，力矩设为0
        Serial.printf("电机%d %s阶梯式力矩衰减完成，力矩已降至0 (总用时: %lums)\n", 
                     motor_index + 1, source, elapsedTime);
        return 0.0f;
    } else {
        // 保持符号，应用当前阶梯的力矩值
        float targetSign = (startValue >= 0) ? 1.0f : -1.0f;
        float resultTorque = currentStepTorque * targetSign;
        
        // 每次进入新阶梯时打印信息 - 使用静态变量区分不同来源
        static uint32_t lastPrintedStep[2][2] = {{0, 0}, {0, 0}}; // [motor_index][source_type]
        int sourceType = (strcmp(source, "状态机") == 0) ? 0 : 1; // 0:状态机, 1:强制静止
        
        if (stepCount != lastPrintedStep[motor_index][sourceType]) {
            lastPrintedStep[motor_index][sourceType] = stepCount;
            // 计算本阶梯的起始值（前一个状态的力矩值）
            float previousStepTorque = 0.0f;
            if (stepCount == 0) {
                // 第1阶梯，起始值就是初始力矩
                previousStepTorque = initialTorque * targetSign;
            } else {
                // 后续阶梯，起始值是上一阶梯的结果
                float prevStepValue = initialTorque - (stepCount * STEP_SIZE);
                previousStepTorque = prevStepValue * targetSign;
            }
            
            Serial.printf("电机%d %s阶梯式衰减 - 第%lu阶梯: %.2f -> %.2f Nm (减少%.1fNm)\n", 
                         motor_index + 1, source, stepCount + 1, 
                         previousStepTorque, resultTorque, abs(previousStepTorque - resultTorque));
        }
        
        return resultTorque;
    }
}

// ==================== 电机位置变化范围计算函数 ===================
/**
 * 计算电机位置变化范围
 * @param channel 电机通道对象
 * @param sampleCount 要分析的样本数量（默认40=2秒，10=500ms）
 * @return 指定时间范围内的位置变化范围
 */
float getMotorPositionRange(MotorChannel& channel, int sampleCount) {
    // 确保样本数量不超过缓冲区大小
    if (sampleCount > ACTIVITY_BUFFER_SIZE) sampleCount = ACTIVITY_BUFFER_SIZE;
    if (sampleCount <= 0) sampleCount = 1;
    
    // 分析指定样本数量内的位置变化范围
    float max_pos = -1000.0f, min_pos = 1000.0f;
    int current_idx = channel.activityBufferIndex;
    
    for (int i = 0; i < sampleCount; i++) {
        current_idx = (current_idx == 0) ? (ACTIVITY_BUFFER_SIZE - 1) : (current_idx - 1);
        if (channel.activityBuffer[current_idx].position > max_pos) max_pos = channel.activityBuffer[current_idx].position;
        if (channel.activityBuffer[current_idx].position < min_pos) min_pos = channel.activityBuffer[current_idx].position;
    }
    
    return max_pos - min_pos;
}

bool gy25t_isStandstill() {
    // 如果窗口未满，不能判断静止状态
    if (!g_gyroWindow.windowFull) {
        return false;
    }
    
    // 检查所有5个点的xyz绝对值是否都小于100
    for (int i = 0; i < GYRO_WINDOW_SIZE; i++) {
        if (abs(g_gyroWindow.x_samples[i]) >= GYRO_STANDSTILL_THRESHOLD ||
            abs(g_gyroWindow.y_samples[i]) >= GYRO_STANDSTILL_THRESHOLD ||
            abs(g_gyroWindow.z_samples[i]) >= GYRO_STANDSTILL_THRESHOLD) {
            return false; // 发现运动
        }
    }
    
    return true; // 所有点都满足静止条件
}

void motorDataCallback(MI_Motor* updated_motor) {
    if (updated_motor->id >= MOTER_1_ID && updated_motor->id <= MOTER_2_ID) {
        int motor_index = updated_motor->id - 1;
        MotorChannel& channel = motorChannels[motor_index];
        channel.motor_obj.position = updated_motor->position;
        channel.motor_obj.velocity = updated_motor->velocity;
        channel.motor_obj.current = updated_motor->current;
        channel.motor_obj.temperature = updated_motor->temperature;
        channel.motor_obj.error = updated_motor->error;
        
        // 更新活动缓冲区
        channel.activityBuffer[channel.activityBufferIndex].position = updated_motor->position;
        channel.activityBuffer[channel.activityBufferIndex].current = updated_motor->current;
        channel.activityBufferIndex = (channel.activityBufferIndex + 1) % ACTIVITY_BUFFER_SIZE;
        
        // 同步数据到BLE管理器
        bleManager.updateMotorData(motor_index, updated_motor->position, updated_motor->current, updated_motor->error);
        bleManager.syncMotorState(motor_index, channel.systemState, channel.detectedActivity, channel.target_torque);
        
        // BLE命令现在直接操作motorChannels，不需要额外同步
        
        // 释放信号量
        if(updated_motor->id == MOTER_1_ID) { 
            xSemaphoreGive(motor1DataReceivedSemaphore); 
        } else { 
            xSemaphoreGive(motor2DataReceivedSemaphore); 
        }
    }
}

// ==================== GPS更新任务 ===================
void gpsUpdateTask(void* parameter) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(200); // 200ms更新频率
    
    for (;;) {
        // 更新GPS数据
        gps_update();
        
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}