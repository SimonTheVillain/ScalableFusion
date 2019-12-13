#include "logger.h"

namespace logging {

Logger *logger = nullptr;

string levelToStr(Level lvl) {
	string loglevel;
	switch(lvl) {
		case Level::STATUS:  loglevel = "[STATUS]";  break;
		case Level::ERROR:   loglevel = "[ERROR]";   break;
		case Level::WARNING: loglevel = "[WARNING]"; break;
		case Level::INFO:    loglevel = "[INFO]";    break;
		case Level::DEBUG:   loglevel = "[DEBUG]";   break;
		default: break;
	}
	return loglevel;
}
void colourStringByLevel(Level lvl, string* msg) {
	switch(lvl) {
		case Level::STATUS:  
			*msg = string("\033[1;32m") + *msg + string("\033[0m"); 
			break;
		case Level::ERROR:   
			*msg = string("\033[1;31m") + *msg + string("\033[0m"); 
			break;
		case Level::WARNING: 
			*msg = string("\033[1;33m") + *msg + string("\033[0m"); 
			break;
		case Level::INFO:    
			*msg = string("\033[1;34m") + *msg + string("\033[0m"); 
			break;
		case Level::DEBUG:   
			*msg = string("\033[1;35m") + *msg + string("\033[0m"); 
			break;
		default: 
			break;
	}
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

	writeLog_(log_msg + string("\n          -> ") + msg);

	colourStringByLevel(lvl, &msg);
	log_msg += string("\n          -> ") + msg;
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