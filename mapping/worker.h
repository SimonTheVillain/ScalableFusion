//
// Created by simon on 11/9/18.
//

#ifndef SUPERMAPPING_WORKER_H
#define SUPERMAPPING_WORKER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>


class Worker{
private:
    std::string debugName;
    bool endThreadVar=false;
    std::mutex taskMutex;
    std::function<void ()> task;
    std::mutex messagingMutex;
    std::condition_variable conditionVar;
    std::thread workerThread;
    static void proc(Worker* worker,std::function<void ()> initializer,std::function<void ()> cleaner);
    void method();
public:
    void setNextTask(std::function<void ()> task);

    void endThread();

    Worker(std::function<void ()> initializer,
            const std::string name,
            std::function<void ()> cleaner =  std::function<void ()>());
    ~Worker();
};


class QueuedWorker{
private:
    bool endThreadVar=false;
    std::mutex tasksMutex;
    std::queue<std::function<void ()>> tasks;
    std::mutex messagingMutex;
    std::condition_variable conditionVar;
    std::thread workerThread;
    static void proc(QueuedWorker *worker, std::function<void ()> initializer);
    void method();
public:
    void appendTask(std::function<void ()> task);

    void endThread();

    QueuedWorker(std::function<void ()> initializer);
    ~QueuedWorker();

};


#endif //SUPERMAPPING_WORKER_H
