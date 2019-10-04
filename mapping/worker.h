#ifndef FILE_WORKER_H
#define FILE_WORKER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>

using namespace std;

class Worker {
public:

	Worker(function<void()> initializer, const string name,
	       function<void()> cleaner = function<void()>());

	~Worker();

	void setNextTask(function<void ()> task);

	void endThread();

private:

	static void proc_(Worker *worker, function<void()> initializer, 
	                  function<void()> cleaner);

	void method_();

	bool end_thread_var_;
	mutex task_mutex_;
	function<void()> task_;
	mutex messaging_mutex_;
	condition_variable condition_var_;
	thread worker_thread_;
};


class QueuedWorker {
public:

	QueuedWorker(function<void()> initializer);

	~QueuedWorker();

	void appendTask(function<void()> task);

	void endThread();

private:

	static void proc_(QueuedWorker *worker, function<void()> initializer);

	void method_();

	bool end_thread_var_;
	mutex tasks_mutex_;
	queue<function<void()>> tasks_;
	mutex messaging_mutex_;
	condition_variable condition_var_;
	thread worker_thread_;
};

#endif
