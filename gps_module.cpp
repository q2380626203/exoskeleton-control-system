#include "gps_module.h"

// 全局GPS数据实例
gps_position_t g_gpsPosition = {0.0f, 0.0f, 0.0f, 'N', 'E', false, 0};
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // 软串口实现

/**
 * 初始化GPS模块
 */
void gps_init() {
    // 初始化GPS软串口
    gpsSerial.begin(GPS_BAUDRATE);
    
    // 初始化GPS数据
    g_gpsPosition.latitude = 0.0f;
    g_gpsPosition.longitude = 0.0f;
    g_gpsPosition.altitude = 0.0f;
    g_gpsPosition.ns = 'N';
    g_gpsPosition.ew = 'E';
    g_gpsPosition.dataValid = false;
    g_gpsPosition.lastUpdateTime = 0;
    
    Serial.printf("GPS模块已初始化 (软串口 RX:GPIO%d, TX:GPIO%d, %d bps)\n", 
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUDRATE);
}

/**
 * 更新GPS数据
 */
void gps_update() {
    static char gpsBuffer[GPS_BUFFER_SIZE];
    static int bufferIndex = 0;
    
    // 读取GPS数据
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        
        if (c == '\n' || c == '\r') {
            // 一行数据结束
            if (bufferIndex > 0) {
                gpsBuffer[bufferIndex] = '\0'; // 添加字符串结束符
                
                // 解析GPS数据
                int parseResult = parse_gps_position(gpsBuffer, &g_gpsPosition);
                if (parseResult > 0) {
                    g_gpsPosition.lastUpdateTime = millis();
                    g_gpsPosition.dataValid = true;
                    
                    // 调试输出
                    static unsigned long lastPrintTime = 0;
                    if (millis() - lastPrintTime > 5000) { // 每5秒打印一次
                        gps_print_position();
                        lastPrintTime = millis();
                    }
                }
                
                // 重置缓冲区
                bufferIndex = 0;
            }
        } else if (bufferIndex < GPS_BUFFER_SIZE - 1) {
            // 添加字符到缓冲区
            gpsBuffer[bufferIndex++] = c;
        } else {
            // 缓冲区满，重置
            bufferIndex = 0;
        }
    }
    
    // 检查数据超时（10秒没有更新认为数据无效）
    if (millis() - g_gpsPosition.lastUpdateTime > 10000) {
        g_gpsPosition.dataValid = false;
    }
}

/**
 * 检查GPS数据是否有效
 */
bool gps_hasValidData() {
    return g_gpsPosition.dataValid;
}

/**
 * 找到NMEA语句中第cx个逗号的位置
 */
int find_comma_pos(char *buf, int cx) {
    char *p = buf;
    int pos = 0;
    
    while(cx > 0 && *p != '\0') {
        if(*p == ',' || *p == '*') {
            if(*p == ',') cx--;
            if(cx == 0) return pos + 1;  // 返回逗号后的位置
        }
        p++;
        pos++;
    }
    return -1;  // 未找到
}

/**
 * 将DDMM.MMMM格式转换为十进制度
 */
float convert_coordinate(char *coord_str) {
    float coord = atof(coord_str);
    int degrees = (int)(coord / 100);      // 度
    float minutes = coord - degrees * 100; // 分
    return degrees + minutes / 60.0;       // 十进制度
}

/**
 * 解析GPS位置数据
 */
int parse_gps_position(char *nmea_data, gps_position_t *pos) {
    char *line = nmea_data;
    char field_buf[20];
    int field_pos;
    
    // 解析GPRMC语句获取经纬度
    if(strstr(line, "$GPRMC") != NULL) {
        // 检查数据状态 (字段2)
        field_pos = find_comma_pos(line, 2);
        if(field_pos > 0 && line[field_pos] != 'A') {
            return 0; // 数据无效
        }
        
        // 纬度 (字段3)
        field_pos = find_comma_pos(line, 3);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->latitude = convert_coordinate(field_buf);
            }
        }
        
        // 南北半球 (字段4)
        field_pos = find_comma_pos(line, 4);
        if(field_pos > 0) {
            pos->ns = line[field_pos];
        }
        
        // 经度 (字段5)
        field_pos = find_comma_pos(line, 5);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->longitude = convert_coordinate(field_buf);
            }
        }
        
        // 东西半球 (字段6)
        field_pos = find_comma_pos(line, 6);
        if(field_pos > 0) {
            pos->ew = line[field_pos];
        }
        
        return 1;  // 成功解析经纬度
    }
    
    // 解析GPGGA语句获取海拔
    if(strstr(line, "$GPGGA") != NULL) {
        // 检查GPS质量指示 (字段6)
        field_pos = find_comma_pos(line, 6);
        if(field_pos > 0 && line[field_pos] == '0') {
            return 0; // GPS质量指示为0，数据无效
        }
        
        // 海拔 (字段9)
        field_pos = find_comma_pos(line, 9);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->altitude = atof(field_buf);
            }
        }
        
        return 2;  // 成功解析海拔
    }
    
    return 0;  // 无关语句
}

/**
 * 打印GPS位置信息
 */
void gps_print_position() {
    if (g_gpsPosition.dataValid) {
        Serial.printf("GPS位置: %.6f°%c, %.6f°%c, 海拔:%.1fm (更新时间:%lu)\n", 
                     g_gpsPosition.latitude, g_gpsPosition.ns,
                     g_gpsPosition.longitude, g_gpsPosition.ew,
                     g_gpsPosition.altitude, g_gpsPosition.lastUpdateTime);
    } else {
        Serial.println("GPS数据无效或未收到数据");
    }
}