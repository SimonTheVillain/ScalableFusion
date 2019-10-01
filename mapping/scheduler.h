#ifndef FILE_SCHEDULER_H
#define FILE_SCHEDULER_H

#include <thread>
#include <condition_variable>
#include <functional>
#include <queue>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "utils/perfMeter.h"

using namespace std;
using namespace Eigen;

class MeshReconstruction;
class Stream;
struct GLFWwindow;
class ActiveSet;

class Worker;

class IncrementalSegmentation;

class Segmentation;

class GarbageCollector;

namespace gfx {
	class GpuTex2D;
} // namespace gfx

//TODO: create a base class for the scheduler
class SchedulerBase {
public:

	virtual ~SchedulerBase() { }

	static GLFWwindow *createConnectedGlContext(GLFWwindow *context);
	static void initializeGlContextInThread(GLFWwindow *context);


	virtual void pause(bool pause) = 0;
	virtual void nextStep() = 0;

	virtual Matrix4f getLastKnownDepthPose() = 0;

};

class SchedulerLinear : public SchedulerBase {
public:

	SchedulerLinear(shared_ptr<MeshReconstruction> map,
	                GarbageCollector *garbage_collector, Stream *capture,
	                GLFWwindow *context,
	                shared_ptr<IncrementalSegmentation> incremental_segmentation);

	~SchedulerLinear();

	//TODO: implement pause and step trough functionality
	void pause(bool pause) {
		paused_ = pause;
	}
	void nextStep() {
		take_next_step_ = true;
	}

	Matrix4f getLastKnownDepthPose() {
		return last_known_depth_pose_;
	}

private:

	void captureWorker_(shared_ptr<MeshReconstruction> map, Stream *stream, 
	                   GLFWwindow *context);

	thread capture_thread_;

	int expand_interval_;

	Matrix4f last_known_depth_pose_;

	bool end_threads_;
	GarbageCollector *garbage_collector_;


	shared_ptr<IncrementalSegmentation> incremental_segmentation_;

	bool paused_;
	bool take_next_step_;
};

#endif
