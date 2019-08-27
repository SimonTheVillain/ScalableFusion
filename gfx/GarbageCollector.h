//
// Created by simon on 3/25/19.
//

#ifndef SUPERMAPPING_GARBAGECOLLECTOR_H
#define SUPERMAPPING_GARBAGECOLLECTOR_H

#include <map>
#include <vector>
#include <mutex>
#include <unordered_map>
#include <thread>
#include <functional>



//TODO: this garbage collector could also be used to handle FBOs
class GarbageCollector {
private:
    std::mutex listMutex;
    std::unordered_map<std::thread::id,std::vector<std::function<void()>>> cleanList;
    std::unordered_map<std::thread::id,std::vector<std::function<void()>>> forceCleanList;


public:

    void Collect();

    //always call this at the end of a thread. this is to ensure that all opengl resources that exist on
    //per thread basis are deleted properly
    void ForceCollect();



    void AddToClean(std::thread::id id,std::function<void()> func);

    void AddToForceCollect(std::function<void()> func);


};


#endif //SUPERMAPPING_GARBAGECOLLECTOR_H
