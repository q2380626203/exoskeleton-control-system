#include "gps_module.h"

// 全局GPS数据实例（基于测试通过的arduino_gps_parser.ino）
gps_position_t g_gpsPosition;
HardwareSerial gpsSerial(GPS_SERIAL_NUM); // 使用UART2硬件串口
static unsigned long g_totalBytesReceived = 0; // GPS接收字节统计

/**
 * 初始化GPS模块 - 使用硬件串口UART2
 */
void gps_init() {
    // 初始化GPS硬件串口
    gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    
    // 初始化GPS数据结构
    memset(&g_gpsPosition, 0, sizeof(g_gpsPosition));
    g_gpsPosition.valid = false;
    g_gpsPosition.lastUpdateTime = 0;
    
    Serial.println("ESP32 GPS数据解析器启动 (基于arduino_gps_parser.ino)");
    Serial.printf("串口配置: %d-8N1, RX=GPIO%d, TX=GPIO%d\n", GPS_BAUDRATE, GPS_RX_PIN, GPS_TX_PIN);
    Serial.println("NMEA校验已启用");
    Serial.println("等待GPS数据...");
}

/**
 * 更新GPS数据 - 基于arduino_gps_parser.ino的逻辑
 */
void gps_update() {
    if (gpsSerial.available()) {
        String nmea = gpsSerial.readStringUntil('\n');
        g_totalBytesReceived += nmea.length() + 1; // +1 for '\n'
        nmea.trim();
        
        // 校验NMEA数据完整性
        if (nmea.length() < 10 || !nmea.startsWith("$")) {
            return;
        }
        
        // 验证NMEA校验和
        if (!validateNMEA(nmea)) {
            Serial.print("校验失败: ");
            Serial.println(nmea);
            return;
        }
        
        if (nmea.startsWith("$GNRMC") || nmea.startsWith("$GPRMC")) {
            parseRMC(nmea);
            if (g_gpsPosition.valid) {
                g_gpsPosition.lastUpdateTime = millis();
                displayGPSData();
            }
        }
        else if (nmea.startsWith("$GNGGA") || nmea.startsWith("$GPGGA")) {
            parseGGA(nmea);
            if (g_gpsPosition.valid) {
                g_gpsPosition.lastUpdateTime = millis();
            }
        }
    }
    
    // 检查数据超时
    if (millis() - g_gpsPosition.lastUpdateTime > 10000) {
        g_gpsPosition.valid = false;
    }
}

/**
 * 检查GPS数据是否有效
 */
bool gps_hasValidData() {
    return g_gpsPosition.valid;
}

// 解析GPRMC数据 (推荐最小定位信息) - 基于arduino_gps_parser.ino
void parseRMC(String rmc) {
    int commaIndex[12];
    int commaCount = 0;
    
    // 查找所有逗号位置
    for (int i = 0; i < rmc.length() && commaCount < 12; i++) {
        if (rmc.charAt(i) == ',') {
            commaIndex[commaCount++] = i;
        }
    }
    
    if (commaCount < 11) return;
    
    // 解析时间 (字段1: HHMMSS.SSS)
    String timeStr = rmc.substring(commaIndex[0] + 1, commaIndex[1]);
    if (timeStr.length() >= 6) {
        g_gpsPosition.hour = timeStr.substring(0, 2).toInt();
        g_gpsPosition.minute = timeStr.substring(2, 4).toInt();
        g_gpsPosition.second = timeStr.substring(4, 6).toInt();
    }
    
    // 解析状态 (字段2: A=有效, V=无效)
    String status = rmc.substring(commaIndex[1] + 1, commaIndex[2]);
    g_gpsPosition.valid = (status == "A");
    
    if (!g_gpsPosition.valid) return;
    
    // 解析纬度 (字段3: DDMM.MMMM)
    String latStr = rmc.substring(commaIndex[2] + 1, commaIndex[3]);
    if (latStr.length() > 0) {
        double lat = latStr.toDouble();
        int degrees = (int)(lat / 100);
        double minutes = lat - (degrees * 100);
        g_gpsPosition.latitude = degrees + minutes / 60.0;
    }
    
    // 解析纬度方向 (字段4: N/S)
    g_gpsPosition.ns = rmc.charAt(commaIndex[3] + 1);
    if (g_gpsPosition.ns == 'S') g_gpsPosition.latitude = -g_gpsPosition.latitude;
    
    // 解析经度 (字段5: DDDMM.MMMM)
    String lonStr = rmc.substring(commaIndex[4] + 1, commaIndex[5]);
    if (lonStr.length() > 0) {
        double lon = lonStr.toDouble();
        int degrees = (int)(lon / 100);
        double minutes = lon - (degrees * 100);
        g_gpsPosition.longitude = degrees + minutes / 60.0;
    }
    
    // 解析经度方向 (字段6: E/W)
    g_gpsPosition.ew = rmc.charAt(commaIndex[5] + 1);
    if (g_gpsPosition.ew == 'W') g_gpsPosition.longitude = -g_gpsPosition.longitude;
    
    // 解析速度 (字段7: 节，转换为km/h)
    String speedStr = rmc.substring(commaIndex[6] + 1, commaIndex[7]);
    if (speedStr.length() > 0) {
        g_gpsPosition.speed = speedStr.toFloat() * 1.852; // 节转km/h
    }
    
    // 解析日期 (字段9: DDMMYY)
    String dateStr = rmc.substring(commaIndex[8] + 1, commaIndex[9]);
    if (dateStr.length() >= 6) {
        g_gpsPosition.day = dateStr.substring(0, 2).toInt();
        g_gpsPosition.month = dateStr.substring(2, 4).toInt();
        g_gpsPosition.year = 2000 + dateStr.substring(4, 6).toInt();
    }
}

// 解析GPGGA数据 (全球定位系统固定数据) - 基于arduino_gps_parser.ino
void parseGGA(String gga) {
    int commaIndex[15];
    int commaCount = 0;
    
    // 查找所有逗号位置
    for (int i = 0; i < gga.length() && commaCount < 15; i++) {
        if (gga.charAt(i) == ',') {
            commaIndex[commaCount++] = i;
        }
    }
    
    if (commaCount < 14) return;
    
    // 解析定位质量 (字段6: 0=无效, 1=GPS, 2=DGPS)
    String qualityStr = gga.substring(commaIndex[5] + 1, commaIndex[6]);
    if (qualityStr.length() > 0) {
        g_gpsPosition.fixQuality = qualityStr.toInt();
    }
    
    // 解析卫星数量 (字段7)
    String satStr = gga.substring(commaIndex[6] + 1, commaIndex[7]);
    if (satStr.length() > 0) {
        g_gpsPosition.satellites = satStr.toInt();
    }
    
    // 解析水平精度因子 (字段8)
    String hdopStr = gga.substring(commaIndex[7] + 1, commaIndex[8]);
    if (hdopStr.length() > 0) {
        g_gpsPosition.hdop = hdopStr.toFloat();
    }
    
    // 解析海拔高度 (字段9)
    String altStr = gga.substring(commaIndex[8] + 1, commaIndex[9]);
    if (altStr.length() > 0) {
        g_gpsPosition.altitude = altStr.toFloat();
    }
}

// 显示GPS数据 - 基于arduino_gps_parser.ino
void displayGPSData() {
    static unsigned long lastDisplay = 0;
    if (millis() - lastDisplay < 2000) return; // 2秒显示一次
    lastDisplay = millis();
    
    Serial.println("==================");
    Serial.println("ESP32 GPS定位数据 (已验证校验和):");
    
    if (g_gpsPosition.valid) {
        Serial.printf("日期: %02d/%02d/%04d\n", g_gpsPosition.day, g_gpsPosition.month, g_gpsPosition.year);
        
        Serial.printf("纬度: %.8f° %c\n", g_gpsPosition.latitude, g_gpsPosition.ns);
        Serial.printf("经度: %.8f° %c\n", g_gpsPosition.longitude, g_gpsPosition.ew);
        
        Serial.printf("海拔: %.1f m\n", g_gpsPosition.altitude);
        
        Serial.printf("定位质量: %d ", g_gpsPosition.fixQuality);
        switch(g_gpsPosition.fixQuality) {
            case 0: Serial.println("(无效)"); break;
            case 1: Serial.println("(GPS)"); break;
            case 2: Serial.println("(DGPS)"); break;
            default: Serial.println("(未知)"); break;
        }
        
        Serial.printf("卫星数: %d\n", g_gpsPosition.satellites);
        Serial.printf("水平精度: %.1f\n", g_gpsPosition.hdop);
               
        // GCJ-02坐标格式（国内地图API）
        double gcjLat, gcjLon;
        wgs84ToGcj02(g_gpsPosition.latitude, g_gpsPosition.longitude, &gcjLat, &gcjLon);
        Serial.println("\nGCJ-02坐标格式(国内地图API):");
        Serial.printf("纬度: %.8f\n", gcjLat);
        Serial.printf("经度: %.8f\n", gcjLon);
        Serial.printf("地图坐标: %.8f,%.8f\n", gcjLat, gcjLon);
        
        
    } else {
        Serial.println("GPS信号无效 - 等待定位...");
    }
    
    Serial.println("==================");
}

/**
 * 打印GPS位置信息 - 兼容接口
 */
void gps_print_position() {
    displayGPSData();
}

// 字符串查找函数
int findCommaPosition(String str, int commaNumber) {
    int count = 0;
    for (int i = 0; i < str.length(); i++) {
        if (str.charAt(i) == ',') {
            count++;
            if (count == commaNumber) {
                return i;
            }
        }
    }
    return -1;
}

// NMEA校验和验证 (强制启用) - 基于arduino_gps_parser.ino
bool validateNMEA(String sentence) {
    // 检查基本格式
    if (sentence.length() < 10 || !sentence.startsWith("$")) {
        return false;
    }
    
    int asteriskPos = sentence.indexOf('*');
    if (asteriskPos == -1) {
        return false;
    }
    
    // 检查校验和长度
    if (sentence.length() < asteriskPos + 3) {
        return false;
    }
    
    String data = sentence.substring(1, asteriskPos); // 去掉$符号
    String checksumStr = sentence.substring(asteriskPos + 1, asteriskPos + 3);
    
    // 计算校验和
    uint8_t calculatedChecksum = 0;
    for (int i = 0; i < data.length(); i++) {
        calculatedChecksum ^= data.charAt(i);
    }
    
    // 转换接收到的校验和
    uint8_t receivedChecksum = (uint8_t)strtol(checksumStr.c_str(), NULL, 16);
    
    bool isValid = (calculatedChecksum == receivedChecksum);
    
    if (!isValid) {
        Serial.printf("校验失败: 计算=%02X, 接收=%02X, 数据=%s\n", 
                     calculatedChecksum, receivedChecksum, data.c_str());
    }
    
    return isValid;
}

/**
 * WGS-84到GCJ-02坐标系转换（火星坐标系）
 * 用于国内地图API（高德、腾讯等）
 */
void wgs84ToGcj02(double wgsLat, double wgsLon, double* gcjLat, double* gcjLon) {
    const double a = 6378245.0; // 长半轴
    const double ee = 0.00669342162296594323; // 偏心率平方
    
    // 判断是否在国外
    if (outOfChina(wgsLat, wgsLon)) {
        *gcjLat = wgsLat;
        *gcjLon = wgsLon;
        return;
    }
    
    double dLat = transformLat(wgsLon - 105.0, wgsLat - 35.0);
    double dLon = transformLon(wgsLon - 105.0, wgsLat - 35.0);
    
    double radLat = wgsLat / 180.0 * PI;
    double magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    double sqrtMagic = sqrt(magic);
    
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
    
    *gcjLat = wgsLat + dLat;
    *gcjLon = wgsLon + dLon;
}

/**
 * 纬度转换辅助函数
 */
double transformLat(double x, double y) {
    double ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

/**
 * 经度转换辅助函数
 */
double transformLon(double x, double y) {
    double ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(abs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
    return ret;
}

/**
 * 判断是否在中国范围外
 */
bool outOfChina(double lat, double lon) {
    if (lon < 72.004 || lon > 137.8347) return true;
    if (lat < 0.8293 || lat > 55.8271) return true;
    return false;
}

/**
 * 显示GPS模块状态和调试信息
 */
void gps_print_status() {
    static unsigned long lastStatusTime = 0;
    
    if (millis() - lastStatusTime > 3000) { // 每3秒更新一次
        lastStatusTime = millis();
        Serial.printf("GPS接收统计: %lu字节\n", g_totalBytesReceived);
    }
}