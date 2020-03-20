#include <logger.h>

int main(void) {
	START_LOGGING(logging::Level::ERROR);

	LOG(logging::Level::DEBUG, "This is a debug message.");
	LOG(logging::Level::ERROR, "This is an error message.");

	STOP_LOGGING();
	
	START_LOGGING(logging::Level::DEBUG);

	LOG(logging::Level::DEBUG, "This is a debug message.");
	LOG(logging::Level::ERROR, "This is an error message.");

	STOP_LOGGING();

	return 0;
}