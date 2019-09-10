#ifndef FILE_GARBAGECOLLECTOR_H
#define FILE_GARBAGECOLLECTOR_H

#include <map>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <functional>

using namespace std;

// TODO: this garbage collector could also be used to handle FBOs
class GarbageCollector {
public:

	void collect();

	// Always call this at the end of a thread. This is to ensure that all openGL 
	// resources that exist on a per thread basis are deleted properly.
	void forceCollect();

	void addToClean(std::thread::id id,std::function<void()> func);

	void addToForceCollect(std::function<void()> func);

private:

	mutex list_mutex_;
	unordered_map<thread::id, vector<function<void()>>> list_clean_;
	unordered_map<thread::id, vector<function<void()>>> list_force_clean_;

};


#endif
