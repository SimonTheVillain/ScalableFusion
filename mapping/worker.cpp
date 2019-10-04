#include "worker.h"

#include <iostream>

using namespace std;

Worker::Worker(function<void()> initializer, const string name,
               function<void()> cleaner) 
		: end_thread_var_(false) {
	worker_thread_ = thread(Worker::proc_, this, initializer, cleaner);
	pthread_setname_np(worker_thread_.native_handle(), name.c_str());
}

Worker::~Worker() {
	endThread();
	worker_thread_.join();
}

void Worker::setNextTask(function<void()> task) {
	task_mutex_.lock();
	task_ = task;
	task_mutex_.unlock();
	condition_var_.notify_one();
}

void Worker::endThread() {
	end_thread_var_ = true;
	condition_var_.notify_one();
}

void Worker::proc_(Worker *worker, function<void()> initializer,
                   function<void()> cleaner) {
	initializer();
	worker->method_();
	if(cleaner) {
		cleaner();
	}
}

void Worker::method_() {
	while(true) {
		unique_lock<mutex> l(messaging_mutex_);
		condition_var_.wait(l);
		if(end_thread_var_) {
			//maybe we also need to call some destructors (opengl contexts and stuff)
			//TODO
			return;
		}

		task_mutex_.lock();
		auto task = task_;
		task_mutex_.unlock();
		if(task) {
			task();
		}
	}
}

QueuedWorker::QueuedWorker(function<void()> initializer)
		: end_thread_var_(false) {
	worker_thread_ = thread(QueuedWorker::proc_, this, initializer);
}

QueuedWorker::~QueuedWorker() {
	endThread();
	worker_thread_.join();
}

void QueuedWorker::appendTask(function<void()> task) {
	tasks_mutex_.lock();
	tasks_.push(task);
	tasks_mutex_.unlock();
	condition_var_.notify_one();
}

void QueuedWorker::endThread() {
	end_thread_var_ = true;
	condition_var_.notify_one();
}

void QueuedWorker::proc_(QueuedWorker *worker, function<void()> initializer) {
	initializer();
	worker->method_();
}

void QueuedWorker::method_() {
	while(true) {
		unique_lock<mutex> l(messaging_mutex_);
		condition_var_.wait(l);

		tasks_mutex_.lock();
		if(end_thread_var_ && tasks_.empty()){
			cout << "TODO: delete the opengl context and stuff here" << endl;
			//maybe we also need to call some destructors (opengl contexts and stuff)
			tasks_mutex_.unlock();
			return;
		}
		tasks_mutex_.unlock();
		while(!tasks_.empty()) {
			tasks_mutex_.lock();
			auto task = tasks_.front();
			tasks_.pop();
			tasks_mutex_.unlock();
			if(task) {
				task();
			}
		}
	}
}
