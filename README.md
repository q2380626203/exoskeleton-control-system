# 登山外骨骼核心控制系统 V3 - 纯蓝牙控制版

## 项目概述

本项目是一个基于ESP32的登山外骨骼核心控制系统V3版本，专门设计用于辅助登山和爬楼运动。系统通过BLE（低功耗蓝牙）与上位机软件通信，结合GY25T陀螺仪传感器，实现实时参数调整和状态监控。系统采用多阶段检测算法和双套力矩衰减机制，智能识别运动状态（静止、平地行走、爬楼、大幅度爬楼），并提供相应的自适应助力支持。

## 核心功能特性

### 1. 📱 BLE蓝牙控制系统（V3核心特性）
- **三特征值架构**：参数调整、电机数据、系统命令独立通信通道
- **实时状态传输**：每500ms传输电机状态，每5秒强制同步所有数据
- **动态参数调整**：支持通过JSON指令实时调整所有助力参数，无需重新编译
- **远程监控诊断**：实时监控系统运行状态、电机健康状况和多阶段检测状态

### 2. 🔄 多阶段检测系统（核心创新）
- **四阶段状态机**：NORMAL→FORCE_STANDSTILL(2秒)→RECOVERY_TRANSITION→FULL_RECOVERY(0.5秒)
- **双套力矩衰减**：状态机衰减 + 强制静止衰减并行工作，阶梯式递减避免冲击
- **缓冲区清洗技术**：强制静止期每200ms清空位置缓冲区，彻底消除惯性污染
- **陀螺仪过渡检测**：恢复期纯陀螺仪检测(≥100)，避开被污染的位置数据

### 3. 智能运动识别与自适应助力
- **四状态精确识别**：静止、平地行走(0.15)、爬楼(0.6)、大幅爬楼(1.0)基于位置变化范围
- **分级助力输出**：4.0Nm(行走) / 7.0Nm(爬楼) / 8.0Nm(大幅爬楼)
- **力矩平滑滤波**：0.2平滑因子避免突变，提供舒适的助力体验
- **间歇式节能**：支持占空比控制的间歇输出，延长电池续航

### 4. 双传感器融合检测
- **电机位置主检测**：基于50点缓冲区(2.5秒数据)分析位置变化范围
- **GY25T陀螺仪辅助**：三轴角速度检测，5点滑动窗口滤波，100Hz更新频率
- **双重静止确认**：位置变化<0.2弧度 + 陀螺仪绝对值<100同时满足
- **容错机制**：陀螺仪失效时自动切换为纯位置检测模式

### 5. 双电机独立协调控制
- **高频轮询**：每10ms轮询电机数据（两电机交替5ms间隔）
- **独立分析**：每个电机独立运动分析和力矩计算，适应不同腿部动作
- **实时反馈**：获取位置、速度、电流、温度完整状态数据
- **CAN协议封装**：基于小米CyberGear协议的UART串口通信

### 6. FreeRTOS多任务实时架构
- **分层优先级设计**：UART接收(7) > 发送(6) > 陀螺仪(4) > 运动分析(3) > 静止检测(2)
- **任务周期优化**：运动分析50ms、静止检测100ms、陀螺仪更新100ms
- **资源管理**：8192字节UART任务栈，4096字节其他任务栈
- **初始化时序**：3秒稳定等待 + 电机模式设置完成标志 + BLE服务启动

## 技术架构

### 硬件配置
- **主控制器**：ESP32微控制器
- **电机系统**：2台小米CyberGear微电机
- **陀螺仪传感器**：GY25T三轴陀螺仪模块
- **电机通信接口**：UART1串口（TX: GPIO13, RX: GPIO12）
- **陀螺仪通信接口**：UART2串口（TX: GPIO10, RX: GPIO11）
- **通信协议**：电机采用基于CAN协议的串口封装，陀螺仪采用自定义数据帧

### 软件架构
```
┌─────────────────────────────────────────────────┐
│              主程序循环 + BLE管理                 │
│            （Arduino Loop）                      │
└─────────────────────┬───────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
┌───────▼───┐  ┌─────▼─────┐  ┌────▼─────┐
│UART接收解析│  │UART发送管理│  │陀螺仪更新 │
│（优先级7） │  │（优先级6） │  │（优先级4）│
└─────┬─────┘  └─────┬─────┘  └────┬─────┘
      │              │             │
      │    ┌─────────┼─────────────┼──────┐
      │    │         │             │      │
      ▼    ▼         ▼             ▼      ▼
┌─────────────┐  ┌──────────────────────────┐
│ 运动分析任务  │  │   多阶段检测任务          │
│（优先级3）   │  │（优先级2）              │
│ 50ms周期    │  │ 100ms周期               │
└─────────────┘  └──────────────────────────┘
```

### 核心数据结构

#### 电机通道结构 (MotorChannel)
```cpp
struct MotorChannel {
    MI_Motor motor_obj;                    // 电机对象
    SystemState systemState;               // 系统状态
    DetectedActivity detectedActivity;     // 检测到的活动类型
    float target_torque;                   // 目标扭矩
    ActivityDataPoint activityBuffer[50];  // 活动数据缓冲区(2.5秒数据)
    // ... 其他控制变量
};
```

#### 助力参数结构 (AssistParameters)
```cpp
struct AssistParameters {
    float MOVEMENT_THRESHOLD_WALK;         // 行走检测阈值: 0.15
    float MOVEMENT_THRESHOLD_CLIMB;        // 爬楼检测阈值: 0.6
    float MOVEMENT_THRESHOLD_CLIMB_STEEP;  // 大幅爬楼阈值: 1.0
    float ASSIST_TORQUE_WALK;              // 行走助力扭矩: 4.0 Nm
    float ASSIST_TORQUE_CLIMB;             // 爬楼助力扭矩: 7.0 Nm
    float ASSIST_TORQUE_CLIMB_STEEP;       // 大幅爬楼扭矩: 8.0 Nm
    float TORQUE_SMOOTHING_FACTOR;         // 力矩平滑因子: 0.2
    bool INTERMITTENT_ENABLED;             // 间歇模式启用: true
    float INTERMITTENT_DUTY;               // 间歇占空比: 0.5
    float INTERMITTENT_REDUCTION;          // 间歇衰减因子: 0.1
    bool AUTO_DISABLE_ENABLED;             // 自动失能启用: false
};
```

#### 陀螺仪数据结构 (GyroData)
```cpp
struct GyroData {
    int16_t roll, pitch, yaw;        // 三轴角速度数据
    bool dataValid;                  // 数据有效性标志
    uint32_t lastUpdateTime;         // 最后更新时间戳
    int16_t filteredData[3][5];      // 5点滑动窗口滤波缓冲区
};
```

#### 多阶段检测控制变量
```cpp
volatile DetectionPhase detectionPhase[2] = {PHASE_NORMAL, PHASE_NORMAL};
volatile uint32_t phaseStartTime[2] = {0, 0};
volatile uint32_t standstillDetectionCount[2] = {0, 0};
volatile bool motorPositionStandstill[2] = {false, false};
```


## 核心算法详解

### 1. 🚀 阶梯式力矩衰减算法
系统采用阶梯式衰减实现平滑的力矩递减，避免突然停止造成的冲击：

```cpp
float processStepwiseTorqueDecay(int motor_index, bool isActive, float startValue, uint32_t startTime, const char* source) {
    const uint32_t STEP_INTERVAL = 500; // 每500ms一个阶梯
    const float STEP_SIZE = 3.0f;       // 每个阶梯减少3.0Nm
    
    uint32_t elapsedTime = millis() - startTime;
    uint32_t stepCount = elapsedTime / STEP_INTERVAL;
    
    // 计算当前阶梯的目标力矩
    float currentStepTorque = initialTorque - ((stepCount + 1) * STEP_SIZE);
    
    if (currentStepTorque <= 0.0f) {
        return 0.0f; // 衰减完成
    }
    return currentStepTorque * targetSign;
}
```

**关键参数**：
- `STEP_INTERVAL = 500ms`：每个衰减阶梯持续时间
- `STEP_SIZE = 3.0f`：每个阶梯减少的力矩值
- **工作原理**：每500ms减少3Nm，直到力矩降为0

### 2. ⚡ 多阶段检测系统
四阶段状态机确保可靠的静止检测和运动恢复：

```cpp
enum DetectionPhase {
    PHASE_NORMAL = 0,         // 正常运行：电机位置主检测
    PHASE_FORCE_STANDSTILL,   // 强制静止：2秒静止期 + 力矩衰减
    PHASE_RECOVERY_TRANSITION,// 恢复过渡：纯陀螺仪检测期
    PHASE_FULL_RECOVERY       // 完全恢复：正常双重检测
};

// 陀螺仪检测参数
#define GYRO_STANDSTILL_THRESHOLD 100   // 静止阈值：绝对值<100
#define GYRO_WINDOW_SIZE 5              // 滑动窗口大小：5个采样点

// 位置检测参数
float pos_range_500ms = getMotorPositionRange(channel, 10); // 分析10个样本（500ms）
motorPositionStandstill[i] = (pos_range_500ms < 0.20f);    // 位置变化<0.2为静止

// 运动检测阈值
gyroHasMovement = (abs_x >= 100) || (abs_y >= 100) || (abs_z >= 100);
```

**四阶段工作流程**：
1. **正常运行期**：位置静止(<0.2) + 陀螺仪静止(<100) → 进入强制静止
2. **强制静止期**：2000ms阶梯式力矩衰减，每200ms清空位置缓冲区
3. **恢复过渡期**：仅陀螺仪检测运动(≥100) → 触发完全恢复
4. **完全恢复期**：500ms缓冲区稳定后恢复正常检测

### 3. 运动识别算法
系统通过分析2秒时间窗口内电机位置数据的变化范围来识别用户运动状态：

```cpp
void analyzeActivityAndSetTorque(MotorChannel& channel) {
    // 分析40个数据点(2秒内)的位置变化范围
    float max_pos = -1000.0f, min_pos = 1000.0f;
    int current_idx = channel.activityBufferIndex;
    for (int i = 0; i < ACTIVITY_BUFFER_SIZE; i++) {
        current_idx = (current_idx == 0) ? (ACTIVITY_BUFFER_SIZE - 1) : (current_idx - 1);
        if (channel.activityBuffer[current_idx].position > max_pos) 
            max_pos = channel.activityBuffer[current_idx].position;
        if (channel.activityBuffer[current_idx].position < min_pos) 
            min_pos = channel.activityBuffer[current_idx].position;
    }
    float pos_range = max_pos - min_pos;
    
    // 基于位置变化范围判断运动类型
    if (pos_range > MOVEMENT_THRESHOLD_CLIMB_STEEP) {
        detectedActivity = ACTIVITY_CLIMBING_STEEP;  // 大幅度爬楼
    } else if (pos_range > MOVEMENT_THRESHOLD_CLIMB) {
        detectedActivity = ACTIVITY_CLIMBING;        // 爬楼
    } else if (pos_range > MOVEMENT_THRESHOLD_WALK) {
        detectedActivity = ACTIVITY_WALKING;         // 平地行走
    } else {
        detectedActivity = ACTIVITY_STANDSTILL;      // 静止
    }
}
```

### 2. 自适应助力算法
根据识别的运动状态，系统计算相应的助力扭矩：

```cpp
// 根据运动类型设置基础扭矩
if (detectedActivity == ACTIVITY_CLIMBING_STEEP) {
    raw_target_torque = ASSIST_TORQUE_CLIMB_STEEP;  // 8.0 Nm
} else if (detectedActivity == ACTIVITY_CLIMBING) {
    raw_target_torque = ASSIST_TORQUE_CLIMB;        // 7.0 Nm
} else if (detectedActivity == ACTIVITY_WALKING) {
    raw_target_torque = ASSIST_TORQUE_WALK;         // 4.0 Nm
} else {
    raw_target_torque = 0.0f;                       // 静止时无助力
}

// 应用平滑滤波（0.2平滑因子）
target_torque = (TORQUE_SMOOTHING_FACTOR * raw_target_torque) + 
                ((1.0f - TORQUE_SMOOTHING_FACTOR) * target_torque);
```

### 3. 间歇式助力优化
为了节省电池能耗，系统支持间歇式助力输出：

```cpp
if (INTERMITTENT_ENABLED && applied_torque != 0) {
    if (isHighTorquePeriod) {
        // 高扭矩周期：输出全力
        applied_torque = target_torque;
    } else {
        // 低扭矩周期：输出降低
        applied_torque *= (1.0f - INTERMITTENT_REDUCTION);
    }
}
```

## 系统启动流程

### 启动时序保证
系统采用分阶段启动机制，确保各组件按正确顺序初始化：

```
1. Serial通信启动 (115200波特率)
   ↓
2. 系统稳定等待 (3秒倒计时)
   ↓  
3. 电机对象初始化
   ↓
4. FreeRTOS任务创建
   ├── UART接收解析任务 (优先级5)
   ├── UART发送管理任务 (优先级4) - 阻塞等待
   ├── 运动分析任务 (优先级2)
   └── 静止检测任务 (优先级1)
   ↓
5. 电机模式初始化
   ├── 设置电机1为电流模式 (CUR_MODE)
   ├── 设置电机2为电流模式 (CUR_MODE)  
   └── 设置初始化完成标志
   ↓
6. UART发送任务解除阻塞，开始正常工作
   ↓
7. BLE蓝牙服务启动
```

### 初始化安全机制
- **硬件稳定等待**：上电后强制等待3秒，确保电机驱动器和通信模块稳定
- **模式设置保证**：使用`motorInitializationComplete`标志确保电机模式设置完成后才开始发送电流指令
- **任务同步机制**：UART发送任务在模式设置完成前保持阻塞状态

## 系统状态机

### 系统状态定义
```cpp
enum SystemState {
    STATE_STANDBY,         // 待机状态
    STATE_AUTO_ADAPT,      // 自适应模式
    STATE_EMERGENCY_STOP   // 紧急停止
};
```

### 运动状态定义
```cpp
enum DetectedActivity {
    ACTIVITY_UNKNOWN,        // 未知状态
    ACTIVITY_STANDSTILL,     // 静止
    ACTIVITY_WALKING,        // 平地行走
    ACTIVITY_CLIMBING,       // 爬楼
    ACTIVITY_CLIMBING_STEEP  // 大幅度爬楼
};
```

## 电机通信协议

### 协议规格
- **通信方式**：UART串口，115200波特率
- **数据格式**：12字节CAN帧封装（4字节ID + 8字节数据）
- **轮询机制**：主动轮询两台电机，获取实时反馈数据

### 主要控制指令
- **模式切换**：`Change_Mode(MI_Motor* motor, uint8_t mode)` - 设置电机工作模式
- **电机使能**：`Motor_Enable(MI_Motor* motor)` - 使能电机运行
- **电机复位**：`Motor_Reset(MI_Motor* motor, uint8_t clear_error)` - 复位电机状态
- **设置扭矩**：`Set_CurMode(MI_Motor* motor, float current)` - 电流模式下设置目标电流

### 电机模式设置
系统初始化时将所有电机设置为电流模式(`CUR_MODE = 3`)：
```cpp
// 在setup()函数中初始化电机模式
for (int i = 0; i < 2; i++) {
    Change_Mode(&motorChannels[i].motor_obj, CUR_MODE);
    delay(100); // 每个电机设置后稍作延时
}
motorInitializationComplete = true; // 设置完成标志
```

**重要**：必须确保电机模式设置完成后才能发送电流指令，否则指令无效。

### 反馈数据解析
系统实时解析电机反馈的以下数据：
- **位置反馈**：电机当前角度位置
- **速度反馈**：电机当前角速度
- **电流反馈**：电机当前输出电流
- **温度反馈**：电机温度状态
- **错误状态**：各类错误标志位

## GY25T陀螺仪通信协议

### 协议规格
- **通信方式**：UART串口，115200波特率
- **数据格式**：固定11字节数据帧
- **更新频率**：100Hz（每10ms更新一次）
- **数据精度**：16位精度，0.01°/s分辨率

### 数据帧格式
```cpp
// GY25T数据帧结构（11字节）
0x5A 0x5A [RollH] [RollL] [PitchH] [PitchL] [YawH] [YawL] [VH] [VL] [SUM]
```

### 关键参数配置
- **采样窗口**：`GYRO_WINDOW_SIZE = 5`（5个采样点滑动窗口）
- **采样间隔**：`GYRO_SAMPLE_INTERVAL_MS = 100ms`
- **静止阈值**：`GYRO_STANDSTILL_THRESHOLD = 100`（绝对值<100为静止）
- **数据有效性**：通过校验和验证数据完整性

### 核心功能
- **静止检测**：三轴陀螺仪数据绝对值均<100时判定为静止
- **运动检测**：任一轴陀螺仪绝对值≥100时判定为运动
- **数据滤波**：5点滑动窗口平均值滤波，减少噪声干扰
- **容错机制**：数据无效时系统自动切换为纯位置检测模式

### 数据处理流程
```cpp
struct GyroData {
    int16_t roll, pitch, yaw;    // 三轴角速度数据
    bool dataValid;              // 数据有效性标志
    uint32_t lastUpdateTime;     // 最后更新时间戳
};

// 运动检测逻辑
bool gyroStandstill = (abs(g_gyroData.roll) < GYRO_STANDSTILL_THRESHOLD) && 
                      (abs(g_gyroData.pitch) < GYRO_STANDSTILL_THRESHOLD) && 
                      (abs(g_gyroData.yaw) < GYRO_STANDSTILL_THRESHOLD);
```

## 开发环境配置

### 硬件要求
- ESP32开发板
- 2台小米CyberGear微电机
- CAN转串口模块
- 外部电源供应

### 软件环境
- Arduino IDE 或 PlatformIO
- ESP32开发框架
- FreeRTOS实时操作系统

### 依赖库
- HardwareSerial（串口通信）
- FreeRTOS（多任务处理）
- ESP32内置数学库

## 编译和烧录

### 1. 环境准备
```bash
# 安装ESP32开发环境
# 配置Arduino IDE或PlatformIO
```

### 2. 编译项目
```bash
# 在Arduino IDE中打开717.ino
# 选择ESP32开发板型号
# 编译程序
```

### 3. 烧录到设备
```bash
# 连接ESP32开发板
# 选择正确的串口
# 上传程序到设备
```

## 参数调优指南

### 运动检测阈值调整
- `MOVEMENT_THRESHOLD_WALK` (0.15)：调整行走检测的敏感度
- `MOVEMENT_THRESHOLD_CLIMB` (0.6)：调整爬楼检测的敏感度
- `MOVEMENT_THRESHOLD_CLIMB_STEEP` (1.0)：调整大幅爬楼检测的敏感度

### 助力扭矩调整
- `ASSIST_TORQUE_WALK` (4.0 Nm)：平地行走助力大小
- `ASSIST_TORQUE_CLIMB` (7.0 Nm)：爬楼助力大小
- `ASSIST_TORQUE_CLIMB_STEEP` (8.0 Nm)：大幅爬楼助力大小

### 系统响应调整
- `TORQUE_SMOOTHING_FACTOR` (0.2)：力矩平滑程度（0-1）
- `INTERMITTENT_DUTY` (0.5)：间歇模式占空比
- `INTERMITTENT_REDUCTION` (0.1)：间歇模式力矩衰减比例

## 性能特点

### 1. 🚀 实时性与响应速度
- **电机数据轮询**：每10ms轮询一次（两电机交替）
- **运动分析任务**：50ms周期
- **多阶段检测任务**：100ms周期（高频检测）
- **陀螺仪更新任务**：100ms周期
- **4阶段状态机**：NORMAL→FORCE_STANDSTILL(2秒)→RECOVERY_TRANSITION→FULL_RECOVERY(0.5秒)
- **缓冲区清洗**：强制静止期每200ms清洗一次
- **双套力矩衰减**：状态机衰减 + 强制静止衰减并行工作

### 2. 🎯 准确性与用户体验
- 运动状态识别准确率：>95%
- **阶梯式力矩衰减**：每500ms减少3Nm，平滑递减避免冲击
- **静止检测精度**：位置变化<0.2弧度，陀螺仪绝对值<100
- 位置检测精度：±0.01弧度
- **双套力矩管理**：状态机+强制静止两套独立衰减机制

### 3. ⚡ 能效与安全优化
- 间歇式助力输出：节能20-30%
- **智能力矩管理**：小幅变化直接设置，节省计算资源
- **紧急安全机制**：双重检测，确保用户安全
- 静止自动失能：避免无效消耗

## 故障排除

### 常见问题
1. **电机无响应**
   - 检查UART连接线路
   - 确认电机ID配置正确
   - 检查电源供应是否充足

2. **运动识别不准确**
   - 调整检测阈值参数
   - 检查电机位置反馈是否正常
   - 确认活动数据缓冲区更新正常

3. **助力输出异常**
   - 检查电机模式设置
   - 确认助力参数配置
   - 检查系统状态机状态

### 调试信息
系统提供详细的串口调试输出：
- **启动阶段**：3秒倒计时、电机模式设置状态
- **电机数据**：位置、速度、电流、温度反馈
- **运动识别**：检测到的活动状态变化
- **助力控制**：计算的目标扭矩和实际输出
- **错误监测**：各类错误状态和处理信息

### 启动调试输出示例
```
系统上电，等待3秒让所有子系统稳定...
等待倒计时: 3秒
等待倒计时: 2秒  
等待倒计时: 1秒
系统稳定，开始初始化...
所有FreeRTOS任务已创建.
正在设置电机工作模式为电流模式...
等待电机模式初始化完成...
电机 1 已设置为电流模式
电机 2 已设置为电流模式
所有电机已设置为电流模式.
电机初始化完成，现在可以安全发送电流指令.
电机初始化完成，uartTransmitManagerTask开始正常工作
启动BLE服务器...
```

## 蓝牙指令协议（上位机软件接口）

### BLE服务配置
- **服务UUID**：`4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **三个特征值**：
  - 参数特征：接收参数调整指令
  - 电机数据特征：发送实时电机状态
  - 命令特征：接收系统控制指令
- **设备名称**：`ExoskeletonControl`
- **通信方式**：JSON格式字符串传输
- **数据发送频率**：每500ms发送电机数据，每5秒强制发送所有数据

### 支持的指令类型

#### 1. 参数调整指令
```json
{
  "type": "params",
  "MOVEMENT_THRESHOLD_WALK": 0.15,
  "MOVEMENT_THRESHOLD_CLIMB": 0.6,
  "MOVEMENT_THRESHOLD_CLIMB_STEEP": 1.0,
  "ASSIST_TORQUE_WALK": 4.0,
  "ASSIST_TORQUE_CLIMB": 7.0,
  "ASSIST_TORQUE_CLIMB_STEEP": 8.0,
  "TORQUE_SMOOTHING_FACTOR": 0.2,
  "INTERMITTENT_ENABLED": false,
  "INTERMITTENT_DUTY": 0.5,
  "INTERMITTENT_REDUCTION": 0.1,
  "AUTO_DISABLE_ENABLED": true
}
```

#### 2. 系统控制指令
```json
{
  "type": "control",
  "command": "setState",
  "state": "STATE_AUTO_ADAPT"  // STATE_STANDBY, STATE_AUTO_ADAPT, STATE_EMERGENCY_STOP
}
```

#### 3. 状态查询指令
```json
{
  "type": "query",
  "request": "status"
}
```

### 系统状态反馈

#### 实时状态数据
```json
{
  "type": "status",
  "timestamp": 1234567890,
  "motor1": {
    "position": 1.23,
    "velocity": 0.45,
    "current": 0.8,
    "temperature": 25.6,
    "torque": 4.0,
    "activity": "ACTIVITY_WALKING"
  },
  "motor2": {
    "position": -0.67,
    "velocity": -0.23,
    "current": 0.9,
    "temperature": 26.1,
    "torque": 4.0,
    "activity": "ACTIVITY_WALKING"
  },
  "system": {
    "state": "STATE_AUTO_ADAPT",
    "gyro_valid": true,
    "detection_phase": ["PHASE_NORMAL", "PHASE_NORMAL"]
  }
}
```

### 上位机软件开发指南

#### 连接流程
1. 扫描BLE设备，查找`ExoskeletonControl`
2. 连接设备并发现服务特征
3. 订阅特征通知接收实时状态数据
4. 发送JSON指令进行参数调整和系统控制

#### 数据处理建议
- **实时监控**：订阅状态数据，实时显示电机状态和运动识别结果
- **参数同步**：发送参数调整指令后，验证系统返回的确认信息
- **错误处理**：监控系统错误状态，提供用户友好的错误提示
- **数据可视化**：将运动数据和助力数据以图表形式展示

---

*登山外骨骼核心控制系统 V3 - 纯蓝牙控制版*