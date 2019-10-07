#include "GarbageCollector.h"

using namespace std;

void GarbageCollector::collect() {
	thread::id id = this_thread::get_id();
	list_mutex_.lock();
	if(list_clean_.count(id) == 1) {
		for(auto func : list_clean_[id]) {
			func();
		}
		list_clean_.erase(id);
	}
	list_mutex_.unlock();
}

void GarbageCollector::forceCollect() {
	thread::id id = this_thread::get_id();
	list_mutex_.lock();
	if(list_force_clean_.count(id) == 1) {
		for(auto func : list_force_clean_[id]) {
			func();
		}
		list_force_clean_.erase(id);
	}
	list_mutex_.unlock();
}

void GarbageCollector::addToClean(thread::id id, std::function<void()> func) {
	list_mutex_.lock();
	list_clean_[id].push_back(func);
	list_mutex_.unlock();
}

void GarbageCollector::addToForceCollect(std::function<void()> func) {
	thread::id id = this_thread::get_id();
	list_mutex_.lock();
	list_force_clean_[id].push_back(func);
	list_mutex_.unlock();
}
