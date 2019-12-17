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

class Logger;
extern Logger *logger;

enum Level {
	STATUS  = 0,
	ERROR   = 1,
	WARNING = 2,
	INFO    = 3,
	DEBUG   = 4
};
string levelToStr(logging::Level lvl);

class Logger {
public:

	Logger(logging::Level lvl)
			: lvl_(lvl) {
		#ifdef LOGFILE 
			name_logfile_ = LOGFILE;
		#endif
		log(logging::Level::STATUS, "Start logging : Level " + levelToStr(lvl));
	}

	~Logger();

	inline void storeLogs(string filename) {
		name_logfile_ = filename;
	}

	void log(logging::Level lvl, string msg, const char *filename = nullptr, int line = 0) {
		#ifdef ENABLE_LOGGING
			genLog_(lvl, msg, filename, line);
		#endif
	}

private:

	void genLog_(logging::Level lvl, string msg, const char *filename, int line);

	void writeLog_(string log_msg);

	logging::Level lvl_;
	string name_logfile_;
};

inline void initLogger(logging::Level lvl) {
	#ifdef ENABLE_LOGGING
		logger = new Logger(lvl);
	#else
		cout << "Logging disabled!" << endl;
	#endif
}

} // namespace logging

#endif // FILE_LOGGER_H