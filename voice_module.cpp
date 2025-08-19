#include "voice_module.h"

VoiceModule::VoiceModule() {
    voiceSerial = &Serial2;
}

void VoiceModule::begin() {
    voiceSerial->begin(9600, SERIAL_8N1, 4, 3);
    delay(1000);
}

void VoiceModule::sendGB2312(const char* text) {
    // 简化的UTF-8到GB2312转换
    // 对于常用汉字，直接使用映射表
    const char* utf8_gb2312_map[][2] = {
        {"系统", "\xCF\xB5\xCD\xB3"},
        {"启动", "\xC6\xF4\xB6\xAF"},
        {"成功", "\xB3\xC9\xB9\xA6"},
        {"助力", "\xD6\xFA\xC1\xA6"},
        {"增加", "\xD4\xF6\xBC\xD3"},
        {"减少", "\xBC\xF5\xC9\xD9"},
        {"当前", "\xB5\xB1\xC7\xB0"},
        {"状态", "\xD7\xB4\xCC\xAC"},
        {"正常", "\xD5\xFD\xB3\xA3"},
        {"零", "\xC1\xE3"}, {"一", "\xD2\xBB"}, {"二", "\xB6\xFE"}, {"三", "\xC8\xFD"},
        {"四", "\xCB\xC4"}, {"五", "\xCE\xE5"}, {"六", "\xC1\xF9"}, {"七", "\xC6\xDF"},
        {"八", "\xB0\xCB"}, {"九", "\xBE\xC5"}, {"十", "\xCA\xAE"},
        {"十一", "\xCA\xAE\xD2\xBB"}, {"十二", "\xCA\xAE\xB6\xFE"}, {"十三", "\xCA\xAE\xC8\xFD"},
        {"十四", "\xCA\xAE\xCB\xC4"}, {"十五", "\xCA\xAE\xCE\xE5"}, {"十六", "\xCA\xAE\xC1\xF9"},
        {"十七", "\xCA\xAE\xC6\xDF"}, {"十八", "\xCA\xAE\xB0\xCB"}, {"十九", "\xCA\xAE\xBE\xC5"},
        {"二十", "\xB6\xFE\xCA\xAE"}
    };
    
    String input = String(text);
    String output = "";
    
    // 查找并替换已知的UTF-8字符
    for (int i = 0; i < sizeof(utf8_gb2312_map) / sizeof(utf8_gb2312_map[0]); i++) {
        input.replace(utf8_gb2312_map[i][0], utf8_gb2312_map[i][1]);
    }
    
    // 直接发送转换后的字节
    voiceSerial->print(input);
}

void VoiceModule::speak(const char* text) {
    sendGB2312(text);
    voiceSerial->print("\r\n\r\n\r\n");
    delay(100);
}

void VoiceModule::speakNumber(int number) {
    String numStr = String(number);
    sendGB2312(numStr.c_str());
    voiceSerial->print("\r\n\r\n\r\n");
    delay(100);
}

void VoiceModule::speakFloat(float number) {
    String numStr = String(number, 2);
    sendGB2312(numStr.c_str());
    voiceSerial->print("\r\n\r\n\r\n");
    delay(100);
}

void VoiceModule::speakStatus(const char* status) {
    sendGB2312("状态：");
    sendGB2312(status);
    voiceSerial->print("\r\n\r\n\r\n");
    delay(100);
}

String VoiceModule::numberToChinese(int number) {
    if (number < 0 || number > 20) {
        return String(number); // 超出范围返回原数字
    }
    
    const char* chineseNumbers[] = {
        "零", "一", "二", "三", "四", "五", "六", "七", "八", "九", "十",
        "十一", "十二", "十三", "十四", "十五", "十六", "十七", "十八", "十九", "二十"
    };
    
    return String(chineseNumbers[number]);
}

void VoiceModule::speakChineseNumber(int number) {
    String chineseNum = numberToChinese(number);
    sendGB2312(chineseNum.c_str());
    voiceSerial->print("\r\n\r\n\r\n");
    delay(100);
}