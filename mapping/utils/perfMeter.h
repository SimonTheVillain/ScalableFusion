#ifndef FILE_PERF_METER_H
#define FILE_PERF_METER_H

#include <chrono>
#include <string>

class FPSCounter{
private:
    std::chrono::time_point<std::chrono::system_clock> last;
public:
    FPSCounter(std::string counterName = "casualCounter");
    ~FPSCounter();
    float printEveryXSeconds=1.0f;
    std::string name;
    bool mute=false;
    int clicks=0;
    void click();
};


#endif
