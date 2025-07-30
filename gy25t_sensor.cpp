#include "gy25t_sensor.h"

// 全局变量定义
GyroRawData g_gyroData = {0, 0, 0, 0, false};
GyroWindowData g_gyroWindow = {{0}, {0}, {0}, {0}, 0, false, 0};
SoftwareSerial gy25tSerial(GY25T_RX_PIN, GY25T_TX_PIN); // 使用软串口

// 数据接收缓冲区
static uint8_t rxBuffer[GY25T_PACKET_SIZE * 2]; // 双倍大小防止溢出
static int bufferIndex = 0;

// 调试打印相关变量
static uint32_t lastPrintTime = 0;
static uint8_t lastRawPacket[GY25T_PACKET_SIZE]; // 保存最后一个完整数据包
static bool hasValidPacket = false;

void gy25t_init() {
    // 初始化软串口
    gy25tSerial.begin(GY25T_BAUDRATE);
    
    // 清空接收缓冲区
    while (gy25tSerial.available()) {
        gy25tSerial.read();
    }
    
    // 初始化数据结构
    g_gyroData.x = 0;
    g_gyroData.y = 0;
    g_gyroData.z = 0;
    g_gyroData.lastUpdateTime = 0;
    g_gyroData.dataValid = false;
    
    // 初始化滑动窗口
    memset(&g_gyroWindow, 0, sizeof(GyroWindowData));
    g_gyroWindow.currentIndex = 0;
    g_gyroWindow.windowFull = false;
    g_gyroWindow.lastSampleTime = 0;
    
    bufferIndex = 0;
    
    Serial.println("GY25T陀螺仪初始化完成 - 使用软串口");
    Serial.printf("软串口配置: 波特率=%d, RX引脚=%d, TX引脚=%d\n", 
                  GY25T_BAUDRATE, GY25T_RX_PIN, GY25T_TX_PIN);
}

void gy25t_update() {
    // 读取串口数据
    while (gy25tSerial.available()) {
        uint8_t receivedByte = gy25tSerial.read();
        
        // 打印接收到的原始字节（十六进制）
        // Serial.printf("GY25T接收: 0x%02X\n", receivedByte);
        
        // 防止缓冲区溢出
        if (bufferIndex >= sizeof(rxBuffer)) {
            bufferIndex = 0;
            Serial.println("GY25T: 缓冲区溢出，重置");
        }
        
        rxBuffer[bufferIndex++] = receivedByte;
        
        // 检查是否有完整数据包
        if (bufferIndex >= GY25T_PACKET_SIZE) {
            // 从缓冲区末尾开始查找完整数据包
            for (int i = bufferIndex - GY25T_PACKET_SIZE; i >= 0; i--) {
                if (rxBuffer[i] == GY25T_FRAME_HEADER_1 && 
                    rxBuffer[i+1] == GY25T_FRAME_HEADER_2 &&
                    rxBuffer[i+2] == GY25T_FRAME_HEADER_3 &&
                    rxBuffer[i+3] == GY25T_FRAME_HEADER_4) {
                    
                    // 找到帧头，打印完整数据包
                    if (i + GY25T_PACKET_SIZE <= bufferIndex) {
                        // Serial.print("GY25T完整数据包: ");
                        // for (int j = 0; j < GY25T_PACKET_SIZE; j++) {
                        //     Serial.printf("0x%02X ", rxBuffer[i + j]);
                        // }
                        // Serial.println();
                        
                        if (gy25t_parsePacket(&rxBuffer[i], GY25T_PACKET_SIZE)) {
                            // 解析成功，移除已处理的数据
                            int remaining = bufferIndex - (i + GY25T_PACKET_SIZE);
                            if (remaining > 0) {
                                memmove(rxBuffer, &rxBuffer[i + GY25T_PACKET_SIZE], remaining);
                            }
                            bufferIndex = remaining;
                            return;
                        }
                    }
                }
            }
            
            // 没找到有效数据包，保留最后10个字节
            if (bufferIndex > 10) {
                memmove(rxBuffer, &rxBuffer[bufferIndex - 10], 10);
                bufferIndex = 10;
            }
        }
    }
}

bool gy25t_parsePacket(uint8_t* buffer, int length) {
    if (length != GY25T_PACKET_SIZE) {
        return false;
    }
    
    // 验证帧头
    if (buffer[0] != GY25T_FRAME_HEADER_1 || 
        buffer[1] != GY25T_FRAME_HEADER_2 ||
        buffer[2] != GY25T_FRAME_HEADER_3 ||
        buffer[3] != GY25T_FRAME_HEADER_4) {
        return false;
    }
    
    // 计算校验和（前10个字节的和）
    uint8_t calculatedChecksum = gy25t_calculateChecksum(buffer, 10);
    uint8_t receivedChecksum = buffer[10];
    
    if (calculatedChecksum != receivedChecksum) {
        Serial.printf("GY25T校验错误: 计算=%02X, 接收=%02X\n", calculatedChecksum, receivedChecksum);
        return false;
    }
    
    // 提取XYZ数据（每2个字节组成一个16位有符号整数）
    // 数据位置：buffer[4-9] 共6个字节，每2个字节一个轴
    // Serial.printf("原始XYZ字节: X=0x%02X%02X, Y=0x%02X%02X, Z=0x%02X%02X\n", 
    //               buffer[4], buffer[5], buffer[6], buffer[7], buffer[8], buffer[9]);
    
    int16_t x = (int16_t)((buffer[4] << 8) | buffer[5]);
    int16_t y = (int16_t)((buffer[6] << 8) | buffer[7]);
    int16_t z = (int16_t)((buffer[8] << 8) | buffer[9]);
    
    // Serial.printf("解析后XYZ值: X=%d, Y=%d, Z=%d\n", x, y, z);
    
    // 更新全局数据
    g_gyroData.x = x;
    g_gyroData.y = y;
    g_gyroData.z = z;
    g_gyroData.lastUpdateTime = millis();
    g_gyroData.dataValid = true;
    
    // 保存原始数据包用于调试打印
    memcpy(lastRawPacket, buffer, GY25T_PACKET_SIZE);
    hasValidPacket = true;
    
    // 更新滑动窗口
    gy25t_updateWindow();
    
    // Serial.println("=== GY25T数据包解析成功 ===");
    return true;
}

uint8_t gy25t_calculateChecksum(uint8_t* data, int length) {
    uint16_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF); // 取低8位
}

void gy25t_printData() {
    if (g_gyroData.dataValid) {
        Serial.printf("陀螺仪数据 - X:%d, Y:%d, Z:%d (更新时间:%lu)\n", 
                      g_gyroData.x, g_gyroData.y, g_gyroData.z, g_gyroData.lastUpdateTime);
    } else {
        Serial.println("陀螺仪数据无效");
    }
}

void gy25t_updateWindow() {
    uint32_t currentTime = millis();
    
    // 检查是否到了采样时间（100ms间隔）
    if (!g_gyroData.dataValid || 
        (g_gyroWindow.lastSampleTime != 0 && 
         currentTime - g_gyroWindow.lastSampleTime < GYRO_SAMPLE_INTERVAL_MS)) {
        return;
    }
    
    // 更新滑动窗口数据
    g_gyroWindow.x_samples[g_gyroWindow.currentIndex] = g_gyroData.x;
    g_gyroWindow.y_samples[g_gyroWindow.currentIndex] = g_gyroData.y;
    g_gyroWindow.z_samples[g_gyroWindow.currentIndex] = g_gyroData.z;
    g_gyroWindow.timestamps[g_gyroWindow.currentIndex] = currentTime;
    
    // 更新索引
    g_gyroWindow.currentIndex = (g_gyroWindow.currentIndex + 1) % GYRO_WINDOW_SIZE;
    
    // 检查窗口是否已满
    if (!g_gyroWindow.windowFull && g_gyroWindow.currentIndex == 0) {
        g_gyroWindow.windowFull = true;
    }
    
    g_gyroWindow.lastSampleTime = currentTime;
}

void gy25t_printDebugData() {
    uint32_t currentTime = millis();
    
    // 检查是否到了打印时间（2秒间隔）
    if (currentTime - lastPrintTime < GY25T_PRINT_INTERVAL_MS) {
        return;
    }
    
    lastPrintTime = currentTime;
    
    // 只打印解析后的数据
    if (g_gyroData.dataValid) {
        Serial.printf("GY25T解析数据 - X:%d, Y:%d, Z:%d\n", 
                      g_gyroData.x, g_gyroData.y, g_gyroData.z);
    } else {
        Serial.println("GY25T解析数据: 无效");
    }
}

