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
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define ACTIVITY_BUFFER_SIZE 50

// ==================== 助力模式参数结构体 (支持动态修改) ===================
struct AssistParameters {
    volatile float MOVEMENT_THRESHOLD_WALK;
    volatile float MOVEMENT_THRESHOLD_CLIMB;
    volatile float MOVEMENT_THRESHOLD_CLIMB_STEEP;
    volatile float ASSIST_TORQUE_WALK;
    volatile float ASSIST_TORQUE_CLIMB;
    volatile float ASSIST_TORQUE_CLIMB_STEEP;
    volatile float TORQUE_SMOOTHING_FACTOR;
    volatile bool INTERMITTENT_ENABLED;
    volatile float INTERMITTENT_DUTY;
    volatile float INTERMITTENT_REDUCTION;
    volatile bool AUTO_DISABLE_ENABLED;
};

AssistParameters assistParams = {
    0.25f, 0.4f, 0.7f, 1.0f, 5.0f, 7.0f, 0.2f, true, 0.5f, 0.2f, false
};

// ==================== 核心数据结构与状态机 ===================
enum UartCommandType { CMD_NONE, CMD_SET_TORQUE, CMD_REQUEST_DATA, CMD_CHANGE_MODE, CMD_ENABLE_MOTOR, CMD_DISABLE_MOTOR };
enum SystemState { STATE_STANDBY, STATE_AUTO_ADAPT, STATE_EMERGENCY_STOP };
enum DetectedActivity { ACTIVITY_UNKNOWN, ACTIVITY_STANDSTILL, ACTIVITY_WALKING, ACTIVITY_CLIMBING, ACTIVITY_CLIMBING_STEEP };
struct ActivityDataPoint { float position; float current; };
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

// 标志变量，用于主循环中处理BLE通信
volatile bool shouldSendParams = false;
volatile bool shouldSendMotorData = false;

// ==================== 函数声明 ===================
void uartReceiveParseTask(void* parameter);
void uartTransmitManagerTask(void* parameter);
void analysisTask(void* parameter);
void analyzeActivityAndSetTorque(MotorChannel& channel);
void motorDataCallback(MI_Motor* updated_motor);
void sendMotorData();
void sendConnectConfirmation();
void sendAllData();
void restartBLE();

// ==================== BLE (蓝牙低功耗) 配置 ===================
// 使用标准的UUID，增加兼容性
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PARAMS_CHAR_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define MOTOR_DATA_CHAR_UUID   "a8c8473c-1481-42a1-a734-71f3a223f3da"
#define COMMAND_CHAR_UUID      "da1a9a2a-e149-4f35-a768-2a1b5351336c"

BLEService *pService = NULL;
BLECharacteristic *pParamsCharacteristic = NULL;
BLECharacteristic *pMotorDataCharacteristic = NULL;
BLECharacteristic *pCommandCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// 这是一个辅助函数，用于将当前参数打包成JSON字符串
void getParamsJson(char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize,
        "{\"walk_thresh\":%.2f, \"climb_thresh\":%.2f, \"steep_thresh\":%.2f, \"walk_torque\":%.2f, \"climb_torque\":%.2f, \"steep_torque\":%.2f, \"smooth_factor\":%.2f, \"intermittent_enabled\":%s, \"intermittent_duty\":%.2f, \"intermittent_reduction\":%.2f, \"auto_disable_enabled\":%s}",
        assistParams.MOVEMENT_THRESHOLD_WALK, assistParams.MOVEMENT_THRESHOLD_CLIMB, assistParams.MOVEMENT_THRESHOLD_CLIMB_STEEP,
        assistParams.ASSIST_TORQUE_WALK, assistParams.ASSIST_TORQUE_CLIMB, assistParams.ASSIST_TORQUE_CLIMB_STEEP,
        assistParams.TORQUE_SMOOTHING_FACTOR, assistParams.INTERMITTENT_ENABLED ? "true" : "false",
        assistParams.INTERMITTENT_DUTY, assistParams.INTERMITTENT_REDUCTION, assistParams.AUTO_DISABLE_ENABLED ? "true" : "false"
    );
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        deviceConnected = true; 
        Serial.println("BLE设备已连接");
        
        // 连接后立即发送一次数据，帮助客户端识别特性
        delay(100); // 给客户端一点时间准备
        
        // 先发送一个简单的连接确认消息
        sendConnectConfirmation();
        
        // 然后设置标志位，主循环会发送正式数据
        shouldSendParams = true;
        shouldSendMotorData = true;
    }
    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        // 蓝牙断开连接时保持当前状态，不切换到待机
        Serial.println("设备断开蓝牙连接，继续保持当前运行状态。");
    }
};

// 添加一个通知回调类，用于监控通知状态
class NotifyCallbacks: public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor* pDescriptor) {
        uint8_t* data = pDescriptor->getValue();
        size_t length = pDescriptor->getLength();
        
        if (length > 0 && data != nullptr && data[0] == 0x01) {
            Serial.println("客户端已订阅通知");
        } else {
            Serial.println("客户端已取消订阅通知");
        }
    }
};

class ParamsCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            int eqIndex = value.indexOf('=');
            if (eqIndex != -1) {
                String key = value.substring(0, eqIndex);
                float val = value.substring(eqIndex + 1).toFloat();
                
                if (key == "walk_thresh") assistParams.MOVEMENT_THRESHOLD_WALK = val;
                else if (key == "climb_thresh") assistParams.MOVEMENT_THRESHOLD_CLIMB = val;
                else if (key == "steep_thresh") assistParams.MOVEMENT_THRESHOLD_CLIMB_STEEP = val;
                else if (key == "walk_torque") assistParams.ASSIST_TORQUE_WALK = val;
                else if (key == "climb_torque") assistParams.ASSIST_TORQUE_CLIMB = val;
                else if (key == "steep_torque") assistParams.ASSIST_TORQUE_CLIMB_STEEP = val;
                else if (key == "smooth_factor") assistParams.TORQUE_SMOOTHING_FACTOR = val;
                else if (key == "auto_disable_enabled") assistParams.AUTO_DISABLE_ENABLED = (val > 0.5);
                else if (key == "intermittent_enabled") assistParams.INTERMITTENT_ENABLED = (val > 0.5);
                else if (key == "intermittent_duty") assistParams.INTERMITTENT_DUTY = constrain(val, 0.0, 1.0);
                else if (key == "intermittent_reduction") assistParams.INTERMITTENT_REDUCTION = constrain(val, 0.0, 1.0);
                
                Serial.printf("通过蓝牙更新参数: %s = %.2f\n", key.c_str(), val);
            }
        }
    }
    
    void onRead(BLECharacteristic *pCharacteristic) {
        Serial.println("收到参数读取请求，发送当前参数");
        // 使用静态缓冲区
        static char paramsJson[512];
        getParamsJson(paramsJson, sizeof(paramsJson));
        pCharacteristic->setValue(paramsJson);
    }
};

class CommandCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            Serial.printf("收到BLE指令: %s\n", value.c_str());

            int separator = value.indexOf(',');
            String cmdName = (separator == -1) ? value : value.substring(0, separator);
            int motor_id = (separator == -1) ? 0 : value.substring(separator + 1).toInt();
            int motor_index = motor_id - 1;

            if (cmdName == "enable_motor" && motor_id >= 1 && motor_id <= 2) {
                if(motorChannels[motor_index].systemState != STATE_EMERGENCY_STOP)
                    motorChannels[motor_index].pending_web_cmd = CMD_ENABLE_MOTOR;
            } else if (cmdName == "disable_motor" && motor_id >= 1 && motor_id <= 2) {
                motorChannels[motor_index].systemState = STATE_STANDBY;
                motorChannels[motor_index].target_torque = 0.0f;
                motorChannels[motor_index].pending_web_cmd = CMD_DISABLE_MOTOR;
            } else if (cmdName == "start_auto_adapt" && motor_id >= 1 && motor_id <= 2) {
                if(motorChannels[motor_index].systemState == STATE_STANDBY){
                    motorChannels[motor_index].systemState = STATE_AUTO_ADAPT;
                    motorChannels[motor_index].pending_web_cmd = CMD_CHANGE_MODE;
                    motorChannels[motor_index].pending_web_value = CUR_MODE;
                }
            } else if (cmdName == "stop_auto_adapt" && motor_id >= 1 && motor_id <= 2) {
                if(motorChannels[motor_index].systemState == STATE_AUTO_ADAPT){
                    motorChannels[motor_index].systemState = STATE_STANDBY;
                    motorChannels[motor_index].target_torque = 0.0f;
                }
            } else if (cmdName == "emergency_stop") {
                for(int i=0; i<2; i++){
                    motorChannels[i].systemState = STATE_EMERGENCY_STOP;
                    motorChannels[i].target_torque = 0.0f;
                    motorChannels[i].pending_web_cmd = CMD_DISABLE_MOTOR;
                }
            } else if (cmdName == "reset_emergency_stop") {
                 for(int i=0; i<2; i++){
                    if (motorChannels[i].systemState == STATE_EMERGENCY_STOP) {
                        motorChannels[i].systemState = STATE_STANDBY;
                    }
                }
            } else if (cmdName == "get_params") {
                // 设置一个标志，让主循环处理参数发送，避免在回调中执行
                shouldSendParams = true;
            } else if (cmdName == "get_motor_data") {
                // 设置一个标志，让主循环处理数据发送，避免在回调中执行
                shouldSendMotorData = true;
            } else if (cmdName == "restart_ble") {
                // 重启BLE连接
                restartBLE();
            }
        }
    }
};

class MotorDataCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
        Serial.println("收到电机数据读取请求，发送当前数据");
        // 不在回调中直接发送数据，而是设置一个标志让主循环处理
        shouldSendMotorData = true;
    }
};

// 辅助函数，用于发送电机数据
void sendMotorData() {
    // 使用静态缓冲区以减少堆栈使用
    static char motor1Json[256];
    static char motor2Json[256];
    static char finalJson[512];
    
    // 确保字符串初始化为空
    motor1Json[0] = '\0';
    motor2Json[0] = '\0';
    finalJson[0] = '\0';
    
    for (int i = 0; i < 2; i++) {
        const char* activityStr = "未知";
        switch (motorChannels[i].detectedActivity) {
            case ACTIVITY_STANDSTILL: activityStr = "静止"; break;
            case ACTIVITY_WALKING: activityStr = "平地行走"; break;
            case ACTIVITY_CLIMBING: activityStr = "爬楼"; break;
            case ACTIVITY_CLIMBING_STEEP: activityStr = "大幅度爬楼"; break;
            default: break;
        }
        char* targetBuffer = (i == 0) ? motor1Json : motor2Json;
        
        // 使用更严格的JSON格式，确保兼容性
        snprintf(targetBuffer, 256,
                "{\"id\":%d,\"position\":%.2f,\"current\":%.2f,\"error\":%d,\"systemState\":%d,\"emergencyStop\":%s,\"detectedActivity\":\"%s\",\"targetTorque\":%.2f}",
                motorChannels[i].motor_obj.id, motorChannels[i].motor_obj.position, motorChannels[i].motor_obj.current, motorChannels[i].motor_obj.error,
                (int)motorChannels[i].systemState, (motorChannels[i].systemState == STATE_EMERGENCY_STOP) ? "true" : "false",
                activityStr, motorChannels[i].target_torque);
    }
    
    // 标准JSON数组格式
    snprintf(finalJson, sizeof(finalJson), "[%s,%s]", motor1Json, motor2Json);
    
    // 确保数据有效
    if (strlen(finalJson) > 10) {
        // 发送数据
        pMotorDataCharacteristic->setValue(finalJson);
        pMotorDataCharacteristic->notify();
        
        // 每10次打印一次调试信息，避免日志过多
        static int debugCounter = 0;
        if (++debugCounter >= 10) {
            debugCounter = 0;
            Serial.printf("电机数据已发送: %s\n", finalJson);
        }
    } else {
        Serial.println("警告: 电机数据无效，跳过发送");
    }
}

// 发送连接确认消息，帮助app识别设备
void sendConnectConfirmation() {
    // 在所有特性上发送简单的确认消息
    const char* confirmMsg = "{\"status\":\"connected\",\"device\":\"exoskeleton\"}";
    
    if(pParamsCharacteristic != nullptr) {
        pParamsCharacteristic->setValue(confirmMsg);
        pParamsCharacteristic->notify();
        delay(50);
    }
    
    if(pMotorDataCharacteristic != nullptr) {
        pMotorDataCharacteristic->setValue(confirmMsg);
        pMotorDataCharacteristic->notify();
    }
    
    Serial.println("已发送连接确认消息");
}

// 添加一个主动发送所有数据的函数
void sendAllData() {
    if (!deviceConnected) return;
    
    Serial.println("主动发送所有数据...");
    
    // 发送参数
    static char paramsJson[512];
    getParamsJson(paramsJson, sizeof(paramsJson));
    pParamsCharacteristic->setValue(paramsJson);
    pParamsCharacteristic->notify();
    delay(50);
    
    // 发送电机数据
    sendMotorData();
    
    Serial.println("所有数据发送完成");
}

void motorDataCallback(MI_Motor* updated_motor) {
    if (updated_motor->id >= MOTER_1_ID && updated_motor->id <= MOTER_2_ID) {
        int motor_index = updated_motor->id - 1;
        MotorChannel& channel = motorChannels[motor_index];
        channel.motor_obj.position = updated_motor->position;
        channel.motor_obj.velocity = updated_motor->velocity;
        channel.motor_obj.current = updated_motor->current;
        // Note: The new MI_Motor struct uses a float for temperature.
        // This might require a cast if the rest of the code expects a uint8_t.
        // For now, we'll leave it as a direct assignment.
        channel.motor_obj.temperature = updated_motor->temperature;
        channel.motor_obj.error = updated_motor->error;
        channel.activityBuffer[channel.activityBufferIndex].position = updated_motor->position;
        channel.activityBuffer[channel.activityBufferIndex].current = updated_motor->current;
        channel.activityBufferIndex = (channel.activityBufferIndex + 1) % ACTIVITY_BUFFER_SIZE;
        if(updated_motor->id == MOTER_1_ID) { xSemaphoreGive(motor1DataReceivedSemaphore); } 
        else { xSemaphoreGive(motor2DataReceivedSemaphore); }
    }
}

// ==================== Arduino Setup ===================
void setup() {
    Serial.begin(115200);
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
    }

    motor1DataReceivedSemaphore = xSemaphoreCreateBinary();
    motor2DataReceivedSemaphore = xSemaphoreCreateBinary();
    UART_Rx_Init(motorDataCallback);

    // 首先创建UART和分析任务
    xTaskCreatePinnedToCore(uartReceiveParseTask, "UartReceiveTask", 8192, NULL, 5, &uartReceiveParseTaskHandle, 1);
    xTaskCreatePinnedToCore(uartTransmitManagerTask, "UartTransmitTask", 8192, NULL, 4, &uartTransmitManagerTaskHandle, 1);
    xTaskCreatePinnedToCore(analysisTask, "AnalysisTask", 8192, NULL, 2, &analysisTaskHandle, 0);
    Serial.println("所有FreeRTOS任务已创建.");
    
    // 增加堆栈大小，配置BLE
    Serial.println("启动BLE服务器...");
    
    // 增加BLE堆栈大小
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.controller_task_stack_size = 4096; // 默认是3072
    esp_bt_controller_init(&bt_cfg);
    
    BLEDevice::init("登山外骨骼-BLE");
    
    // 设置MTU大小，确保数据包能够完整发送
    BLEDevice::setMTU(512);
    
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    // 创建服务前先清理旧服务
    if(pService != nullptr) {
        pService->stop();
        delete pService;
    }
    
    // 创建新服务
    pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);  // 增加属性表大小

    pParamsCharacteristic = pService->createCharacteristic(
        PARAMS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
    );
    pParamsCharacteristic->setCallbacks(new ParamsCharacteristicCallbacks());
    BLE2902* params2902 = new BLE2902();
    params2902->setCallbacks(new NotifyCallbacks());
    // 不使用setNotifications，因为这个API可能不存在
    pParamsCharacteristic->addDescriptor(params2902);

    pMotorDataCharacteristic = pService->createCharacteristic(
        MOTOR_DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
    );
    pMotorDataCharacteristic->setCallbacks(new MotorDataCharacteristicCallbacks());
    BLE2902* motorData2902 = new BLE2902();
    motorData2902->setCallbacks(new NotifyCallbacks());
    // 不使用setNotifications，因为这个API可能不存在
    pMotorDataCharacteristic->addDescriptor(motorData2902);

    pCommandCharacteristic = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandCharacteristic->setCallbacks(new CommandCharacteristicCallbacks());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 连接的优先级，帮助iPhone连接
    pAdvertising->setMinPreferred(0x12);  // 连接的优先级，帮助iPhone连接
    
    // 设置广播数据以增强兼容性
    BLEAdvertisementData advData;
    advData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode
    advData.setCompleteServices(BLEUUID(SERVICE_UUID));
    
    // 使用一个简短且易识别的设备名
    advData.setName("外骨骼-BLE");
    
    // 添加制造商数据，包含一个简单标识符，帮助app识别
    uint8_t manData[4] = {0xA1, 0xB2, 0xC3, 0xD4}; // 唯一标识
    advData.setManufacturerData(std::string((char*)manData, 4));
    
    pAdvertising->setAdvertisementData(advData);
    
    // 设置扫描响应数据，提供更多信息
    BLEAdvertisementData scanResponse;
    scanResponse.setName("登山外骨骼-BLE");
    
    // 可以添加其他信息，如电源类型、版本号等
    uint8_t powerData[1] = {0x03}; // 0x03 表示使用电池供电
    scanResponse.setServiceData(BLEUUID((uint16_t)0x180F), std::string((char*)powerData, 1)); // 0x180F 是电池服务UUID
    
    pAdvertising->setScanResponseData(scanResponse);
    
    BLEDevice::startAdvertising();
    Serial.println("BLE广播已开始。");
}

// 重启BLE连接的函数
void restartBLE() {
    Serial.println("正在重启BLE连接...");
    
    // 停止广播
    BLEDevice::stopAdvertising();
    
    // 断开所有连接
    if (deviceConnected) {
        // 通知客户端我们即将断开
        pMotorDataCharacteristic->setValue("正在重启BLE连接，请稍后重新连接");
        pMotorDataCharacteristic->notify();
        delay(500);
        
        // 强制断开连接
        deviceConnected = false;
        oldDeviceConnected = false;
    }
    
    // 重新启动广播
    BLEDevice::startAdvertising();
    Serial.println("BLE已重启，等待新的连接...");
}

// 在loop函数中添加自动重连机制
void loop() {
    // 添加BLE连接监控
    static unsigned long lastConnectedTime = 0;
    static bool wasConnected = false;
    static unsigned long lastForceSendTime = 0;
    
    if (deviceConnected) {
        lastConnectedTime = millis();
        wasConnected = true;
        
        // 每5秒主动发送一次所有数据，确保app能接收到
        if (millis() - lastForceSendTime > 5000) {
            sendAllData();
            lastForceSendTime = millis();
        }
    } else if (wasConnected && millis() - lastConnectedTime > 10000) {
        // 如果曾经连接过，但断开超过10秒，尝试重启BLE
        wasConnected = false;
        restartBLE();
    }
    
    // 修改蓝牙连接检测逻辑
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // 给BLE堆栈时间处理断开事件
        oldDeviceConnected = deviceConnected;
        Serial.println("设备已断开蓝牙连接");
        BLEDevice::startAdvertising();
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        Serial.println("设备已通过蓝牙连接");
        shouldSendParams = true; // 连接时发送参数
    }

    // 处理BLE通信标志
    if (deviceConnected) {
        // 处理参数发送请求
        if (shouldSendParams) {
            static char paramsJson[512];
            getParamsJson(paramsJson, sizeof(paramsJson));
            pParamsCharacteristic->setValue(paramsJson);
            pParamsCharacteristic->notify();
            Serial.println("已通过蓝牙发送参数。");
            shouldSendParams = false;
        }
        
        // 处理电机数据发送请求
        if (shouldSendMotorData) {
            sendMotorData();
            Serial.println("已通过蓝牙发送电机数据。");
            shouldSendMotorData = false;
        }
        
        // 定期发送电机数据
        static unsigned long lastNotifyTime = 0;
        if (millis() - lastNotifyTime > 500) {
            sendMotorData();
            lastNotifyTime = millis();
        }
    }
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
    
    for (;;) {
        MotorChannel& channel = motorChannels[next_motor_to_poll - 1];
        MI_Motor* motor_to_poll = &channel.motor_obj;
        SemaphoreHandle_t semaphore_to_wait_for = (next_motor_to_poll == MOTER_1_ID) ? motor1DataReceivedSemaphore : motor2DataReceivedSemaphore;
        xSemaphoreTake(semaphore_to_wait_for, (TickType_t)0);
        
        // 1. 处理一次性指令 (来自Web/蓝牙)
        if (channel.pending_web_cmd != CMD_NONE) {
            switch(channel.pending_web_cmd) {
                case CMD_ENABLE_MOTOR: 
                    Motor_Enable(motor_to_poll); 
                    break;
                case CMD_DISABLE_MOTOR: 
                    Motor_Reset(motor_to_poll, 0); // The new Motor_Reset function requires a second argument.
                    break;
                case CMD_CHANGE_MODE: 
                    Change_Mode(motor_to_poll, (uint8_t)channel.pending_web_value);
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

        if (xSemaphoreTake(semaphore_to_wait_for, pdMS_TO_TICKS(150)) != pdTRUE) {
            Serial.printf("电机 %d 响应超时.\n", next_motor_to_poll);
        }
        next_motor_to_poll = (next_motor_to_poll == MOTER_1_ID) ? MOTER_2_ID : MOTER_1_ID;
    }
}

void uartReceiveParseTask(void* parameter) {
    for (;;) {
        handle_uart_rx();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void analyzeActivityAndSetTorque(MotorChannel& channel) {
    float max_pos = -1000.0f, min_pos = 1000.0f;
    int current_idx = channel.activityBufferIndex;
    for (int i = 0; i < ACTIVITY_BUFFER_SIZE; i++) {
        current_idx = (current_idx == 0) ? (ACTIVITY_BUFFER_SIZE - 1) : (current_idx - 1);
        if (channel.activityBuffer[current_idx].position > max_pos) max_pos = channel.activityBuffer[current_idx].position;
        if (channel.activityBuffer[current_idx].position < min_pos) min_pos = channel.activityBuffer[current_idx].position;
    }
    float pos_range = max_pos - min_pos;
    float raw_target_torque = 0.0f;
    DetectedActivity previousActivity = channel.detectedActivity;

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
        raw_target_torque = 0.0f;
        channel.detectedActivity = ACTIVITY_STANDSTILL;
    }

    if (previousActivity != channel.detectedActivity) {
        if (previousActivity == ACTIVITY_STANDSTILL && channel.detectedActivity != ACTIVITY_STANDSTILL) {
            channel.activityChanged = true;
            channel.activityChangeTime = millis();
            channel.lastActivity = previousActivity;
        }
    }

    channel.target_torque = (assistParams.TORQUE_SMOOTHING_FACTOR * raw_target_torque) + ((1.0f - assistParams.TORQUE_SMOOTHING_FACTOR) * channel.target_torque);
    if (abs(channel.target_torque) < 0.01) {
        channel.target_torque = 0.0f;
    }
}
