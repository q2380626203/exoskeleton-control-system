#ifndef GY25T_SENSOR_H
#define GY25T_SENSOR_H

#include <Arduino.h>
#include <HardwareSerial.h>

// GY25T数据包结构定义
#define GY25T_FRAME_HEADER_1    0xA4
#define GY25T_FRAME_HEADER_2    0x03
#define GY25T_FRAME_HEADER_3    0x0E
#define GY25T_FRAME_HEADER_4    0x06
#define GY25T_PACKET_SIZE       11
#define GY25T_DATA_SIZE         6
#define GY25T_CHECKSUM_SIZE     1

// 串口配置
#define GY25T_BAUDRATE          115200
#define GY25T_TX_PIN            11
#define GY25T_RX_PIN            10
#define GY25T_SERIAL_NUM        2  // 使用UART2，避免与电机UART1冲突

// 陀螺仪原始数据结构
struct GyroRawData {
    int16_t x;
    int16_t y; 
    int16_t z;
    uint32_t lastUpdateTime;
    bool dataValid;
};

// 陀螺仪滑动窗口配置
#define GYRO_WINDOW_SIZE 5
#define GYRO_SAMPLE_INTERVAL_MS 100
#define GYRO_STANDSTILL_THRESHOLD 100

// 陀螺仪滑动窗口数据结构
struct GyroWindowData {
    int16_t x_samples[GYRO_WINDOW_SIZE];
    int16_t y_samples[GYRO_WINDOW_SIZE];
    int16_t z_samples[GYRO_WINDOW_SIZE];
    uint32_t timestamps[GYRO_WINDOW_SIZE];
    int currentIndex;
    bool windowFull;
    uint32_t lastSampleTime;
};

// 全局变量声明
extern GyroRawData g_gyroData;
extern GyroWindowData g_gyroWindow;
extern HardwareSerial gy25tSerial;

// 函数声明
void gy25t_init();
void gy25t_update();
bool gy25t_parsePacket(uint8_t* buffer, int length);
uint8_t gy25t_calculateChecksum(uint8_t* data, int length);
void gy25t_printData();
void gy25t_updateWindow();

#endif // GY25T_SENSOR_H