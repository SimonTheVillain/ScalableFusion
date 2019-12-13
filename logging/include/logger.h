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
Logger *logger = nullptr;

enum Level {
	STATUS  = 0,
	ERROR   = 1,
	WARNING = 2,
	INFO    = 3,
	DEBUG   = 4
};
string levelToStr(int lvl) {
	string loglevel;
	switch(lvl) {
		case Level::STATUS:  loglevel = "[STATUS]";  break;
		case Level::ERROR:   loglevel = "[ERROR]";   break;
		case Level::WARNING: loglevel = "[WARNING"; break;
		case Level::INFO:    loglevel = "[INFO]";    break;
		case Level::DEBUG:   loglevel = "[DEBUG]";   break;
		default: break;
	}
	return loglevel;
}

class Logger {
public:

	Logger(Level lvl) 
			: lvl_(lvl) {
		#ifdef LOGFILE 
			name_logfile_ = LOGFILE;
		#endif
		log(Level::STATUS, "Start logging : Level " + levelToStr(lvl));
	}

	~Logger() {
		LOG(Level::STATUS, "Stop logging\n");
	}

	inline void storeLogs(string filename) {
		name_logfile_ = filename;
	}

	void log(Level lvl, string msg, const char *filename = nullptr, int line = 0) {
		#ifdef ENABLE_LOGGING
			if(lvl > lvl_)
				return;

			auto now = time(nullptr);
			auto time_str = string(ctime(&now));
			time_str.pop_back();

			string loglevel = levelToStr(lvl);
			loglevel.insert(loglevel.end(), 10 - loglevel.size(), ' ');

			string log_msg(loglevel + time_str);
			if(filename) 
				log_msg += string(" - ") + string(filename);
			if(line != 0) 
				log_msg	+= string(": ") + to_string(line);

			log_msg += string("\n  -> ") + msg;

			writeLog_(log_msg);

			cout << log_msg << endl;
		#endif
	}

private:

	void writeLog_(string log_msg) {
		if(!name_logfile_.empty()) {
			ofstream logfile;
			logfile.open(name_logfile_, ios_base::app);
			if(!logfile.is_open()) {
				LOG(Level::WARNING, "Could not open logfile " + name_logfile_);
			} else {
				logfile << log_msg << endl;
				logfile.close();
			}
		}
	}

	Level lvl_;
	string name_logfile_;
};

void initLogger(Level lvl) {
	#ifdef ENABLE_LOGGING
		logger = new Logger(lvl);
	#else
		cout << "Logging disabled!" << endl;
	#endif
}

} // namespace logging

#endif // FILE_LOGGER_H