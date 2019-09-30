#ifndef FILE_PERF_METER_H
#define FILE_PERF_METER_H

#include <chrono>
#include <string>

using namespace std;

class FPSCounter {
public:

	FPSCounter(string counter_name = "casualCounter");

	void click();

	string name;
	float print_every_x_seconds;
	bool mute;
	int clicks;
	
private:

	chrono::time_point<chrono::system_clock> last_;

};

#endif
