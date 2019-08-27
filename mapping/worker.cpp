//
// Created by simon on 11/9/18.
//

#include "worker.h"
#include <iostream>

using namespace std;


void Worker::proc(Worker *worker, std::function<void ()> initializer,std::function<void ()> cleaner)
{
    initializer();
    worker->method();
    if(cleaner){
        cleaner();
    }
}

void Worker::method()
{
    while(true){
        unique_lock<mutex> l(messagingMutex);
        conditionVar.wait(l);
        if(endThreadVar){
            //maybe we also need to call some destructors (opengl contexts and stuff)
            //TODO
            return;
        }

        taskMutex.lock();
        auto t = task;
        taskMutex.unlock();
        if(t){
            t();
        }
    }
}

void Worker::setNextTask(std::function<void ()> task)
{
    taskMutex.lock();
    this->task = task;
    taskMutex.unlock();
    conditionVar.notify_one();
}

void Worker::endThread()
{
    endThreadVar = true;
    conditionVar.notify_one();
}

Worker::Worker(std::function<void ()> initializer,const std::string name,std::function<void ()> cleaner)
{
    workerThread = thread(Worker::proc,this,initializer,cleaner);
    //renameThread(workerThread,name);
    pthread_setname_np(workerThread.native_handle(),name.c_str());
    debugName=name;

}

Worker::~Worker()
{
    endThread();

    workerThread.join();
}

void QueuedWorker::proc(QueuedWorker *worker, std::function<void ()> initializer)
{
    initializer();
    worker->method();
}

void QueuedWorker::method()
{

    while(true){
        unique_lock<mutex> l(messagingMutex);
        conditionVar.wait(l);


        tasksMutex.lock();
        if(endThreadVar &&
           this->tasks.empty()){
            cout << "TODO: delete the opengl context and stuff here" << endl;
            //maybe we also need to call some destructors (opengl contexts and stuff)
            tasksMutex.unlock();
            return;
        }
        tasksMutex.unlock();
        while(!tasks.empty()){
            tasksMutex.lock();
            auto t = this->tasks.front();
            this->tasks.pop();
            tasksMutex.unlock();
            if(t){
                t();
            }
        }
    }
}

void QueuedWorker::appendTask(std::function<void ()> task)
{

    tasksMutex.lock();
    this->tasks.push(task);
    tasksMutex.unlock();
    conditionVar.notify_one();
}

void QueuedWorker::endThread()
{
    endThreadVar = true;
    conditionVar.notify_one();

}

QueuedWorker::QueuedWorker(std::function<void ()> initializer)
{
    workerThread = thread(QueuedWorker::proc,this,initializer);

}

QueuedWorker::~QueuedWorker()
{
    endThread();

    workerThread.join();
}

