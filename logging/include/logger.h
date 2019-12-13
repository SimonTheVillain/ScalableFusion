#ifndef FILE_LOGGER_H
#define FILE_LOGGER_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>

using namespace std;

#define START_LOGGING(lvl) {logging::initLogger(lvl);}
#define LOG(lvl, msg)      {logging::logger->log(lvl, msg, __FILE__, __LINE__);}
#define STOP_LOGGING()     {delete logging::logger;}

namespace logging {

enum Level {
	STATUS  = 0,
	ERROR   = 1,
	WARNING = 2,
	INFO    = 3,
	DEBUG   = 4
};
string levelToStr(int lvl);

class Logger {
public:

	Logger(Level lvl);

	~Logger();

	inline void storeLogs(string filename) {
		name_logfile_ = filename;
	}

	void log(Level lvl, string msg, const char *filename = nullptr, int line = 0);

private:

	void writeLog_(string log_msg);

	Level lvl_;
	string name_logfile_;
};

void initLogger(Level lvl);

extern Logger *logger;

} // namespace logging

#endif // FILE_LOGGER_H