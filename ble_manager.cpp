#include "ble_manager.h"

// 全局实例
BLEManager bleManager;

// BLE回调类定义
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { 
        bleManager.deviceConnected = true; 
        Serial.println("BLE设备已连接");
        
        // 连接后立即发送一次数据，帮助客户端识别特性
        delay(100); // 给客户端一点时间准备
        
        // 先发送一个简单的连接确认消息
        bleManager.sendConnectConfirmation();
        
        // 然后设置标志位，主循环会发送正式数据
        bleManager.setShouldSendParams(true);
        bleManager.setShouldSendMotorData(true);
    }
    
    void onDisconnect(BLEServer* pServer) {
        bleManager.deviceConnected = false;
        // 蓝牙断开连接时保持当前状态，不切换到待机
        Serial.println("设备断开蓝牙连接，继续保持当前运行状态。");
    }
};

class NotifyCallbacks: public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor* pDescriptor) {
        uint8_t* data = pDescriptor->getValue();
        size_t length = pDescriptor->getLength();
        
        if (length >= 2) {
            bool notificationsEnabled = (data[0] == 0x01);
            Serial.printf("通知状态变更: %s\n", notificationsEnabled ? "启用" : "禁用");
        }
    }
};

class ParamsCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            int eqIndex = value.indexOf('=');
            if (eqIndex > 0) {
                String key = value.substring(0, eqIndex);
                float val = value.substring(eqIndex + 1).toFloat();
                Serial.printf("收到参数更新: %s = %.3f\n", key.c_str(), val);
                
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
                
                bleManager.setShouldSendParams(true);
            }
        }
    }
    
    void onRead(BLECharacteristic *pCharacteristic) {
        Serial.println("收到参数读取请求，发送当前参数");
        bleManager.setShouldSendParams(true);
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
                if(getMotorSystemState(motor_index) != STATE_EMERGENCY_STOP) {
                    setMotorPendingCommand(motor_index, CMD_ENABLE_MOTOR, 0);
                    Serial.printf("收到电机%d使能指令\n", motor_id);
                } else {
                    Serial.printf("错误: 电机%d处于紧急停止状态，无法使能\n", motor_id);
                }
            } else if (cmdName == "disable_motor" && motor_id >= 1 && motor_id <= 2) {
                setMotorSystemState(motor_index, STATE_STANDBY);
                setMotorTargetTorque(motor_index, 0.0f);
                setMotorPendingCommand(motor_index, CMD_DISABLE_MOTOR, 0);
                Serial.printf("收到电机%d禁用指令\n", motor_id);
            } else if (cmdName == "start_auto_adapt" && motor_id >= 1 && motor_id <= 2) {
                if(getMotorSystemState(motor_index) == STATE_STANDBY){
                    setMotorSystemState(motor_index, STATE_AUTO_ADAPT);
                    setMotorPendingCommand(motor_index, CMD_CHANGE_MODE, 3); // CUR_MODE
                    Serial.printf("电机 %d 切换到自适应模式 (直接设置主程序状态)\n", motor_id);
                } else {
                    Serial.printf("错误: 电机%d不在待机状态，当前状态=%d\n", motor_id, (int)getMotorSystemState(motor_index));
                }
            } else if (cmdName == "emergency_stop") {
                Serial.println("收到紧急停止指令");
                for(int i=0; i<2; i++){
                    setMotorSystemState(i, STATE_EMERGENCY_STOP);
                    setMotorTargetTorque(i, 0.0f);
                    setMotorPendingCommand(i, CMD_DISABLE_MOTOR, 0);
                }
            } else if (cmdName == "reset_emergency_stop") {
                for(int i=0; i<2; i++){
                    if (getMotorSystemState(i) == STATE_EMERGENCY_STOP) {
                        setMotorSystemState(i, STATE_STANDBY);
                    }
                }
                Serial.println("紧急停止已重置，所有电机切换到待机模式");
            } else if (cmdName == "stop_auto_adapt" && motor_id >= 1 && motor_id <= 2) {
                if(getMotorSystemState(motor_index) == STATE_AUTO_ADAPT){
                    setMotorSystemState(motor_index, STATE_STANDBY);
                    setMotorTargetTorque(motor_index, 0.0f);
                    Serial.printf("电机 %d 停止自适应模式\n", motor_id);
                } else {
                    Serial.printf("错误: 电机%d未处于自适应模式\n", motor_id);
                }
            } else if (cmdName == "get_params") {
                // 设置一个标志，让主循环处理参数发送，避免在回调中执行
                bleManager.setShouldSendParams(true);
                Serial.println("收到参数请求指令");
            } else if (cmdName == "get_motor_data") {
                // 设置一个标志，让主循环处理数据发送，避免在回调中执行
                bleManager.setShouldSendMotorData(true);
                Serial.println("收到电机数据请求指令");
            } else if (cmdName == "get_all_data") {
                // 请求发送所有数据
                bleManager.setShouldSendParams(true);
                bleManager.setShouldSendMotorData(true);
                Serial.println("收到全部数据请求指令");
            } else if (cmdName == "restart_ble") {
                // 重启BLE连接
                Serial.println("收到BLE重启指令");
                bleManager.restart();
            } else if (cmdName == "set_torque" && motor_id >= 1 && motor_id <= 2) {
                // 手动设置扭矩 格式: set_torque,motor_id,torque_value
                int secondSeparator = value.indexOf(',', separator + 1);
                if (secondSeparator != -1) {
                    float torque_value = value.substring(secondSeparator + 1).toFloat();
                    bleManager.motorData[motor_index].target_torque = torque_value;
                    Serial.printf("电机 %d 手动设置扭矩: %.2f Nm\n", motor_id, torque_value);
                }
            } else if (cmdName == "get_system_status") {
                // 发送系统状态信息
                String statusMsg = "System OK, Motors: ";
                statusMsg += (bleManager.motorData[0].systemState == STATE_AUTO_ADAPT) ? "M1:AUTO " : "M1:STANDBY ";
                statusMsg += (bleManager.motorData[1].systemState == STATE_AUTO_ADAPT) ? "M2:AUTO" : "M2:STANDBY";
                Serial.println("发送系统状态: " + statusMsg);
                // 可以通过特性发送状态信息
            } else if (cmdName == "ping") {
                // 连接测试指令
                Serial.println("收到Ping指令，系统正常响应");
                bleManager.setShouldSendMotorData(true); // 发送数据作为响应
            } else if (cmdName == "help") {
                // 帮助指令 - 输出所有可用指令
                Serial.println("=== BLE指令帮助 ===");
                Serial.println("电机控制:");
                Serial.println("  enable_motor,<1|2>     - 启用电机");
                Serial.println("  disable_motor,<1|2>    - 禁用电机");
                Serial.println("  start_auto_adapt,<1|2> - 启动自适应模式");
                Serial.println("  stop_auto_adapt,<1|2>  - 停止自适应模式");
                Serial.println("  set_torque,<1|2>,<value> - 手动设置扭矩");
                Serial.println("安全控制:");
                Serial.println("  emergency_stop         - 紧急停止");
                Serial.println("  reset_emergency_stop   - 重置紧急停止");
                Serial.println("数据查询:");
                Serial.println("  get_params            - 获取参数");
                Serial.println("  get_motor_data        - 获取电机数据");
                Serial.println("  get_all_data          - 获取所有数据");
                Serial.println("  get_system_status     - 获取系统状态");
                Serial.println("系统控制:");
                Serial.println("  restart_ble           - 重启蓝牙");
                Serial.println("  ping                  - 连接测试");
                Serial.println("  help                  - 显示此帮助");
            } else {
                Serial.printf("未识别的BLE指令: %s (输入'help'查看可用指令)\n", cmdName.c_str());
            }
        }
    }
};

class MotorDataCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
        Serial.println("收到电机数据读取请求，发送当前数据");
        bleManager.setShouldSendMotorData(true);
    }
};

// BLEManager实现
BLEManager::BLEManager() : 
    pService(nullptr),
    pParamsCharacteristic(nullptr),
    pMotorDataCharacteristic(nullptr),
    pCommandCharacteristic(nullptr),
    deviceConnected(false),
    oldDeviceConnected(false),
    shouldSendParams(false),
    shouldSendMotorData(false) {
}

void BLEManager::init() {
    Serial.println("启动BLE服务器...");
    
    // 增加BLE堆栈大小
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    bt_cfg.controller_task_stack_size = 4096;
    esp_bt_controller_init(&bt_cfg);
    
    BLEDevice::init("登山外骨骼-BLE");
    BLEDevice::setMTU(512);
    
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    
    if (pService) {
        pService->stop();
        pServer->removeService(pService);
        delete pService;
        pService = nullptr;
    }
    
    pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);

    // 参数特性
    pParamsCharacteristic = pService->createCharacteristic(
        PARAMS_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | 
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE
    );
    pParamsCharacteristic->setCallbacks(new ParamsCharacteristicCallbacks());
    BLE2902* params2902 = new BLE2902();
    params2902->setCallbacks(new NotifyCallbacks());
    pParamsCharacteristic->addDescriptor(params2902);

    // 电机数据特性
    pMotorDataCharacteristic = pService->createCharacteristic(
        MOTOR_DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | 
        BLECharacteristic::PROPERTY_INDICATE
    );
    pMotorDataCharacteristic->setCallbacks(new MotorDataCharacteristicCallbacks());
    BLE2902* motorData2902 = new BLE2902();
    motorData2902->setCallbacks(new NotifyCallbacks());
    pMotorDataCharacteristic->addDescriptor(motorData2902);

    // 命令特性
    pCommandCharacteristic = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandCharacteristic->setCallbacks(new CommandCharacteristicCallbacks());

    pService->start();

    // 广播设置
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    
    BLEAdvertisementData advData;
    advData.setFlags(0x06);
    advData.setCompleteServices(BLEUUID(SERVICE_UUID));
    advData.setName("外骨骼-BLE");
    
    uint8_t manData[4] = {0xA1, 0xB2, 0xC3, 0xD4};
    advData.setManufacturerData(std::string((char*)manData, 4));
    pAdvertising->setAdvertisementData(advData);
    
    BLEAdvertisementData scanResponse;
    scanResponse.setName("登山外骨骼-BLE");
    
    uint8_t powerData[1] = {0x03};
    scanResponse.setServiceData(BLEUUID((uint16_t)0x180F), std::string((char*)powerData, 1));
    pAdvertising->setScanResponseData(scanResponse);
    
    BLEDevice::startAdvertising();
    Serial.println("BLE广播已开始。");
}

void BLEManager::handleLoop() {
    if (!deviceConnected && oldDeviceConnected) {
        delay(500);
        BLEDevice::startAdvertising();
        Serial.println("设备已断开蓝牙连接，重新开始广播等待连接");
        oldDeviceConnected = deviceConnected;
    }
    
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
    }
    
    if (shouldSendParams) {
        sendParams();
        shouldSendParams = false;
    }
    
    if (shouldSendMotorData) {
        sendMotorData();
        shouldSendMotorData = false;
    }
}

void BLEManager::restart() {
    Serial.println("重启BLE连接...");
    if (pService) {
        pService->stop();
    }
    BLEDevice::deinit(true);
    delay(1000);
    init();
}

void BLEManager::sendParams() {
    if (deviceConnected && pParamsCharacteristic) {
        static char paramsJson[512];
        getParamsJson(paramsJson, sizeof(paramsJson));
        pParamsCharacteristic->setValue(paramsJson);
        pParamsCharacteristic->notify();
    }
}

void BLEManager::sendMotorData() {
    if (deviceConnected && pMotorDataCharacteristic) {
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
            switch (motorData[i].detectedActivity) {
                case ACTIVITY_STANDSTILL: activityStr = "静止"; break;
                case ACTIVITY_WALKING: activityStr = "平地行走"; break;
                case ACTIVITY_CLIMBING: activityStr = "爬楼"; break;
                case ACTIVITY_CLIMBING_STEEP: activityStr = "大幅度爬楼"; break;
                default: break;
            }
            char* targetBuffer = (i == 0) ? motor1Json : motor2Json;
            
            // 使用与备份文件相同的JSON格式，确保兼容性
            snprintf(targetBuffer, 256,
                    "{\"id\":%d,\"position\":%.2f,\"current\":%.2f,\"error\":%d,\"systemState\":%d,\"emergencyStop\":%s,\"detectedActivity\":\"%s\",\"targetTorque\":%.2f}",
                    i + 1, // 电机ID (1或2)
                    motorData[i].position, 
                    motorData[i].current, 
                    motorData[i].error, // 使用真实的错误信息
                    (int)motorData[i].systemState, 
                    (motorData[i].systemState == STATE_EMERGENCY_STOP) ? "true" : "false",
                    activityStr, 
                    motorData[i].target_torque);
        }
        
        // 标准JSON数组格式，与备份文件保持一致
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
            Serial.println("警告: 电机数据格式无效，未发送");
        }
    }
}

void BLEManager::updateMotorData(int motorIndex, float position, float current, int error) {
    if (motorIndex >= 0 && motorIndex < 2) {
        motorData[motorIndex].position = position;
        motorData[motorIndex].current = current;
        motorData[motorIndex].error = error;
    }
}

void BLEManager::syncMotorState(int motorIndex, SystemState state, DetectedActivity activity, float torque) {
    if (motorIndex >= 0 && motorIndex < 2) {
        motorData[motorIndex].systemState = state;
        motorData[motorIndex].detectedActivity = activity;
        motorData[motorIndex].target_torque = torque;
    }
}

UartCommandType BLEManager::getPendingCommand(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 2) {
        return motorData[motorIndex].pending_web_cmd;
    }
    return CMD_NONE;
}

float BLEManager::getPendingWebValue(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 2) {
        return motorData[motorIndex].pending_web_value;
    }
    return 0.0f;
}

void BLEManager::clearPendingCommand(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 2) {
        motorData[motorIndex].pending_web_cmd = CMD_NONE;
        motorData[motorIndex].pending_web_value = 0.0f;
    }
}

SystemState BLEManager::getMotorSystemState(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 2) {
        return motorData[motorIndex].systemState;
    }
    return STATE_STANDBY;
}

float BLEManager::getMotorTargetTorque(int motorIndex) {
    if (motorIndex >= 0 && motorIndex < 2) {
        return motorData[motorIndex].target_torque;
    }
    return 0.0f;
}

void BLEManager::sendConnectConfirmation() {
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

void BLEManager::sendAllData() {
    sendParams();
    delay(100);
    sendMotorData();
}

void BLEManager::sendCommandResponse(const String& response) {
    // 简化命令响应，主要通过状态更新反映
    // 触发数据更新作为响应
    setShouldSendMotorData(true);
}

void BLEManager::getParamsJson(char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize,
        "{\"walk_thresh\":%.2f, \"climb_thresh\":%.2f, \"steep_thresh\":%.2f, \"walk_torque\":%.2f, \"climb_torque\":%.2f, \"steep_torque\":%.2f, \"smooth_factor\":%.2f, \"intermittent_enabled\":%s, \"intermittent_duty\":%.2f, \"intermittent_reduction\":%.2f, \"auto_disable_enabled\":%s}",
        assistParams.MOVEMENT_THRESHOLD_WALK, assistParams.MOVEMENT_THRESHOLD_CLIMB, assistParams.MOVEMENT_THRESHOLD_CLIMB_STEEP,
        assistParams.ASSIST_TORQUE_WALK, assistParams.ASSIST_TORQUE_CLIMB, assistParams.ASSIST_TORQUE_CLIMB_STEEP,
        assistParams.TORQUE_SMOOTHING_FACTOR, assistParams.INTERMITTENT_ENABLED ? "true" : "false",
        assistParams.INTERMITTENT_DUTY, assistParams.INTERMITTENT_REDUCTION, assistParams.AUTO_DISABLE_ENABLED ? "true" : "false"
    );
}