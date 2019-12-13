#include "logger.h"

namespace logging {

Logger *logger = nullptr;

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

Logger::~Logger() {
	LOG(Level::STATUS, "Stop logging\n");
}

void Logger::genLog_(Level lvl, string msg, const char *filename, int line) {
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
}

void Logger::writeLog_(string log_msg) {
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

} // namespace logging