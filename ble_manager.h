#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// 包含主程序的数据结构（避免循环依赖）
enum UartCommandType { CMD_NONE, CMD_SET_TORQUE, CMD_REQUEST_DATA, CMD_CHANGE_MODE, CMD_ENABLE_MOTOR, CMD_DISABLE_MOTOR };
enum SystemState { STATE_STANDBY, STATE_AUTO_ADAPT, STATE_EMERGENCY_STOP };
enum DetectedActivity { ACTIVITY_UNKNOWN, ACTIVITY_STANDSTILL, ACTIVITY_WALKING, ACTIVITY_CLIMBING, ACTIVITY_CLIMBING_STEEP };

struct ActivityDataPoint { 
    float position; 
    float current; 
};

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

// 简化的MotorChannel结构（只包含BLE需要的字段）
struct BLEMotorData {
    volatile SystemState systemState;
    volatile DetectedActivity detectedActivity;
    volatile float target_torque;
    volatile UartCommandType pending_web_cmd;
    volatile float pending_web_value;
    float position;  // 实际电机位置
    float current;   // 实际电机电流
    int error;       // 电机错误状态
};

// BLE UUID定义
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define PARAMS_CHAR_UUID       "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define MOTOR_DATA_CHAR_UUID   "a8c8473c-1481-42a1-a734-71f3a223f3da"
#define COMMAND_CHAR_UUID      "da1a9a2a-e149-4f35-a768-2a1b5351336c"

// BLE管理器类
class BLEManager {
public:
    BLEManager();
    
    // 初始化和管理
    void init();
    void handleLoop();
    void restart();
    
    // 数据发送
    void sendParams();
    void sendMotorData();
    void sendConnectConfirmation();
    void sendAllData();
    void sendCommandResponse(const String& response);
    
    // 状态获取
    bool isConnected() const { return deviceConnected; }
    bool wasConnected() const { return oldDeviceConnected; }
    
    // 标志设置（由外部模块调用）
    void setShouldSendParams(bool value) { shouldSendParams = value; }
    void setShouldSendMotorData(bool value) { shouldSendMotorData = value; }
    
    // 数据更新（由主程序调用，更新电机实时数据）
    void updateMotorData(int motorIndex, float position, float current, int error = 0);
    
    // 电机状态同步接口
    void syncMotorState(int motorIndex, SystemState state, DetectedActivity activity, float torque);
    UartCommandType getPendingCommand(int motorIndex);
    float getPendingWebValue(int motorIndex);
    void clearPendingCommand(int motorIndex);
    SystemState getMotorSystemState(int motorIndex);
    float getMotorTargetTorque(int motorIndex);
    
    // 为了友元访问
    friend class MyServerCallbacks;
    friend class ParamsCharacteristicCallbacks;
    friend class CommandCharacteristicCallbacks;
    friend class MotorDataCharacteristicCallbacks;
    
private:
    // BLE对象
    BLEService *pService;
    BLECharacteristic *pParamsCharacteristic;
    BLECharacteristic *pMotorDataCharacteristic;
    BLECharacteristic *pCommandCharacteristic;
    
    // 连接状态
    bool deviceConnected;
    bool oldDeviceConnected;
    
    // 发送标志
    volatile bool shouldSendParams;
    volatile bool shouldSendMotorData;
    
    // 电机数据存储
    BLEMotorData motorData[2];
    
    // 辅助函数
    void getParamsJson(char* buffer, size_t bufferSize);
};

// 全局BLE管理器实例
extern BLEManager bleManager;

// 外部引用（需要在主程序中定义）
extern AssistParameters assistParams;

// 主程序接口函数声明（在717.ino中实现）
void setMotorSystemState(int motor_index, SystemState state);
void setMotorPendingCommand(int motor_index, UartCommandType cmd, float value);
SystemState getMotorSystemState(int motor_index);
void setMotorTargetTorque(int motor_index, float torque);

#endif // BLE_MANAGER_H