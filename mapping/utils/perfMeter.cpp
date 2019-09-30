#include "perfMeter.h"

#include <iostream>

using namespace std;

FPSCounter::FPSCounter(string counter_name) 
		: name(counter_name),
		  print_every_x_seconds(1.0f),
		  mute(false),
		  clicks(0) {
	last_ = std::chrono::system_clock::now();
}

void FPSCounter::click() {
	clicks++;
	auto now = chrono::system_clock::now();
	auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - last_);

	if(elapsed.count() / 1000.0 > print_every_x_seconds) {
		if(!mute) {
			cout << name << " at " << 
			        float(clicks) / double(elapsed.count()) * 1000.0 <<
			        "fps" << endl;
		}
		clicks = 0;
		last_ = now;
	}
}
