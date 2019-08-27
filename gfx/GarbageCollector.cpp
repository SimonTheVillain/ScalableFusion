//
// Created by simon on 3/25/19.
//

#include "GarbageCollector.h"


using namespace std;

void GarbageCollector::Collect() {
    thread::id id = this_thread::get_id();
    listMutex.lock();
    if(cleanList.count(id) == 1){
        for(auto func : cleanList[id]){
            func();
        }
        cleanList.erase(id);
    }
    listMutex.unlock();
}

void GarbageCollector::ForceCollect(){

    thread::id id = this_thread::get_id();
    listMutex.lock();
    if(forceCleanList.count(id) == 1){
        for(auto func : forceCleanList[id]){
            func();
        }
        forceCleanList.erase(id);
    }
    listMutex.unlock();
}


void GarbageCollector::AddToClean(thread::id id,std::function<void()> func) {
    listMutex.lock();
    cleanList[id].push_back(func);
    listMutex.unlock();
}
void GarbageCollector::AddToForceCollect(std::function<void()> func) {
    thread::id id = this_thread::get_id();
    listMutex.lock();
    forceCleanList[id].push_back(func);
    listMutex.unlock();
}

