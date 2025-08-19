#ifndef VOICE_MODULE_H
#define VOICE_MODULE_H

#include <HardwareSerial.h>

class VoiceModule {
private:
    HardwareSerial* voiceSerial;
    void sendGB2312(const char* text);
    String numberToChinese(int number);
    
public:
    VoiceModule();
    void begin();
    void speak(const char* text);
    void speakNumber(int number);
    void speakFloat(float number);
    void speakStatus(const char* status);
    void speakChineseNumber(int number);
};

#endif