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
    g_gpsPosition.satellites = 0;
    g_gpsPosition.hdop = 0.0f;
    g_gpsPosition.fixQuality = 0;
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
    static unsigned long totalLinesProcessed = 0;
    
    // 读取GPS数据
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        
        if (c == '\n' || c == '\r') {
            // 一行数据结束
            if (bufferIndex > 0) {
                gpsBuffer[bufferIndex] = '\0'; // 添加字符串结束符
                totalLinesProcessed++;
                
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
    
    // 解析GPRMC/GNRMC语句获取经纬度
    if(strstr(line, "$GPRMC") != NULL || strstr(line, "$GNRMC") != NULL) {
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
    
    // 解析GPGGA/GNGGA语句获取海拔、卫星数量和精度
    if(strstr(line, "$GPGGA") != NULL || strstr(line, "$GNGGA") != NULL) {
        // 检查GPS质量指示 (字段6)
        field_pos = find_comma_pos(line, 6);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            pos->fixQuality = atoi(field_buf);
            if (pos->fixQuality == 0) {
                return 0; // GPS质量指示为0，数据无效
            }
        }
        
        // 使用的卫星数量 (字段7)
        field_pos = find_comma_pos(line, 7);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->satellites = atoi(field_buf);
            }
        }
        
        // 水平精度稀释值HDOP (字段8)
        field_pos = find_comma_pos(line, 8);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->hdop = atof(field_buf);
            }
        }
        
        // 海拔 (字段9)
        field_pos = find_comma_pos(line, 9);
        if(field_pos > 0) {
            sscanf(line + field_pos, "%[^,]", field_buf);
            if (strlen(field_buf) > 0) {
                pos->altitude = atof(field_buf);
            }
        }
        
        return 2;  // 成功解析海拔和精度信息
    }
    
    return 0;  // 无关语句
}

/**
 * 打印GPS位置信息
 */
void gps_print_position() {
    if (g_gpsPosition.dataValid) {
        Serial.printf("GPS位置: %.6f°%c, %.6f°%c, 海拔:%.1fm\n", 
                     g_gpsPosition.latitude, g_gpsPosition.ns,
                     g_gpsPosition.longitude, g_gpsPosition.ew,
                     g_gpsPosition.altitude);
        
        // 显示定位精度和卫星信息
        Serial.printf("定位精度: %.1fm (%s), 使用卫星: %d颗\n",
                     g_gpsPosition.hdop,
                     (g_gpsPosition.hdop < 2.0) ? "优秀" : 
                     (g_gpsPosition.hdop < 5.0) ? "良好" : "一般",
                     g_gpsPosition.satellites);
        
        // 显示定位质量
        const char* fixType[] = {"无效", "GPS", "DGPS", "PPS"};
        Serial.printf("定位类型: %s, 更新时间: %lu\n",
                     (g_gpsPosition.fixQuality < 4) ? fixType[g_gpsPosition.fixQuality] : "未知",
                     g_gpsPosition.lastUpdateTime);
    } else {
        // 详细的无信号状态分析
        Serial.println("=== GPS状态: 无有效定位信号 ===");
        
        // 检查数据超时情况
        uint32_t timeSinceUpdate = millis() - g_gpsPosition.lastUpdateTime;
        if (g_gpsPosition.lastUpdateTime == 0) {
            Serial.println("状态: 从未收到GPS数据");
            Serial.println("检查: 1) GPS模块是否连接到GPIO4");
            Serial.println("     2) GPS模块是否正常供电(3.3V)");
            Serial.println("     3) 波特率是否为9600");
        } else if (timeSinceUpdate > 10000) {
            Serial.printf("状态: GPS数据超时 (%lu秒前最后更新)\n", timeSinceUpdate/1000);
            Serial.println("检查: 1) GPS模块连接是否松动");
            Serial.println("     2) 软串口是否被其他任务占用");
        } else {
            Serial.printf("状态: 收到GPS数据但无有效定位 (%lu秒前更新)\n", timeSinceUpdate/1000);
        }
        
        // 显示当前GPS状态信息
        if (g_gpsPosition.satellites > 0) {
            Serial.printf("当前状态: 搜索到%d颗卫星，但信号不足定位\n", g_gpsPosition.satellites);
            Serial.println("建议: 移动到室外开阔区域，等待卫星信号稳定");
        } else {
            Serial.println("当前状态: 未搜索到任何卫星信号");
            Serial.println("建议: 1) 移动到室外空旷区域");
            Serial.println("     2) 检查天线连接(ANTENNA OPEN问题)");
            Serial.println("     3) 等待冷启动完成(可能需要数分钟)");
        }
        
        // 定位质量分析
        if (g_gpsPosition.fixQuality == 0) {
            Serial.println("定位质量: 无定位");
        } else {
            const char* fixType[] = {"无效", "GPS", "DGPS", "PPS"};
            Serial.printf("定位质量: %s (但数据无效)\n", 
                         (g_gpsPosition.fixQuality < 4) ? fixType[g_gpsPosition.fixQuality] : "未知");
        }
        
        Serial.println("提示: GPS首次启动或长时间未使用需要较长时间获取卫星信息");
        Serial.println("===============================");
    }
}

/**
 * 显示GPS模块状态和调试信息
 */
void gps_print_status() {
    static unsigned long totalBytesReceived = 0;
    static unsigned long totalLinesProcessed = 0;
    static unsigned long lastStatusTime = 0;
    
    // 统计接收到的数据
    while (gpsSerial.available()) {
        gpsSerial.read();
        totalBytesReceived++;
    }
    
    if (millis() - lastStatusTime > 1000) { // 每秒更新一次
        lastStatusTime = millis();
        
        Serial.println("=== GPS模块状态 ===");
        Serial.printf("软串口: GPIO%d, %d bps\n", GPS_RX_PIN, GPS_BAUDRATE);
        Serial.printf("接收统计: %lu字节, %lu行\n", totalBytesReceived, totalLinesProcessed);
        
        if (totalBytesReceived == 0) {
            Serial.println("⚠️  未接收到任何数据");
            Serial.println("检查: GPS模块连接、供电、波特率");
        } else if (totalBytesReceived < 100) {
            Serial.println("⚠️  接收数据量很少");
            Serial.println("检查: GPS模块是否正常工作");
        } else {
            Serial.println("✅ GPS模块通信正常");
            
            if (g_gpsPosition.dataValid) {
                Serial.println("✅ GPS定位成功");
            } else {
                Serial.println("⏳ GPS搜星中，等待定位...");
            }
        }
        Serial.println("==================");
    }
}