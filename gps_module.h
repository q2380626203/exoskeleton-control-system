#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <HardwareSerial.h>

// GPS数据结构（基于测试通过的arduino_gps_parser.ino）
typedef struct {
    bool valid;
    double latitude;
    double longitude;
    float altitude;
    float speed;
    int satellites;
    int hour, minute, second;
    int day, month, year;
    char ns, ew;
    int fixQuality;
    float hdop;
    uint32_t lastUpdateTime; // 最后更新时间
} gps_position_t;

// GPS配置 - 使用硬件串口UART2
#define GPS_RX_PIN 10       // ESP32 GPIO11 接收GPS模块TX
#define GPS_TX_PIN 11       // ESP32 GPIO10 发送到GPS模块RX
#define GPS_BAUDRATE 9600   // GPS模块波特率
#define GPS_SERIAL_NUM 2    // 使用UART2
#define GPS_BUFFER_SIZE 256 // GPS数据缓冲区大小

// 全局GPS数据
extern gps_position_t g_gpsPosition;
extern HardwareSerial gpsSerial;

// 函数声明
void gps_init();
void gps_update();
bool gps_hasValidData();
void gps_print_position();
void gps_print_status();

// 内部函数
bool validateNMEA(String sentence);
void parseRMC(String rmc);
void parseGGA(String gga);
void displayGPSData();
int findCommaPosition(String str, int commaNumber);

// 坐标系转换函数
void wgs84ToGcj02(double wgsLat, double wgsLon, double* gcjLat, double* gcjLon);
double transformLat(double x, double y);
double transformLon(double x, double y);  
bool outOfChina(double lat, double lon);

#endif // GPS_MODULE_H