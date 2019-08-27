#include "perfMeter.h"
#include <iostream>

using namespace std;

FPSCounter::FPSCounter(std::string counterName)
{
    this->name=counterName;
    last = std::chrono::system_clock::now();
}

FPSCounter::~FPSCounter()
{

}

void FPSCounter::click()
{
    clicks++;
    auto now = std::chrono::system_clock::now();
    auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last);
    if(elapsed.count()/1000.0>printEveryXSeconds){
        if(!mute){
            cout << name << " at " << float(clicks)/double(elapsed.count())*1000.0 <<
                    "fps" << endl;
        }
        clicks=0;
        last = now;
    }
}
