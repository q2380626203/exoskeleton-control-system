#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// GPS数据结构
typedef struct {
    float latitude;    // 纬度 (十进制度)
    float longitude;   // 经度 (十进制度) 
    float altitude;    // 海拔 (米)
    char ns;           // 南北半球标识 ('N' or 'S')
    char ew;           // 东西半球标识 ('E' or 'W')
    int satellites;    // 使用的卫星数量
    float hdop;        // 水平精度稀释（定位精度）
    int fixQuality;    // 定位质量 (0=无效, 1=GPS, 2=DGPS)
    bool dataValid;    // 数据有效性标志
    uint32_t lastUpdateTime; // 最后更新时间
} gps_position_t;

// GPS配置
#define GPS_RX_PIN 4        // ESP32 GPIO4 接收GPS模块TX
#define GPS_TX_PIN 2        // ESP32 GPIO2 发送到GPS模块RX (可以不用)
#define GPS_BAUDRATE 9600   // GPS模块波特率
#define GPS_BUFFER_SIZE 256 // GPS数据缓冲区大小

// 全局GPS数据
extern gps_position_t g_gpsPosition;
extern SoftwareSerial gpsSerial;

// 函数声明
void gps_init();
void gps_update();
bool gps_hasValidData();
int find_comma_pos(char *buf, int cx);
float convert_coordinate(char *coord_str);
int parse_gps_position(char *nmea_data, gps_position_t *pos);
void gps_print_position();

#endif // GPS_MODULE_H