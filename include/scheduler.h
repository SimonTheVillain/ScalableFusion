#ifndef FILE_SCHEDULER_H
#define FILE_SCHEDULER_H

#include <thread>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class MeshReconstruction;
namespace video {
	class Source;
}
struct GLFWwindow;
class ActiveSet;

class Worker;
class GpuStorage;


class Segmentation;

class GarbageCollector;

class LowDetailRenderer;

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
					GpuStorage* gpu_storage, video::Source *source,
	                GLFWwindow *context,
	                LowDetailRenderer *low_detail_renderer);

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

	void captureWorker_(shared_ptr<MeshReconstruction> reconstruction, 
	                    video::Source *source, GLFWwindow *context);

	thread capture_thread_;

	int expand_interval_;

	Matrix4f last_known_depth_pose_;

	bool end_threads_;
	GpuStorage *gpu_storage_;



	LowDetailRenderer* low_detail_renderer_;
	bool paused_;
	bool take_next_step_;
};

#endif // FILE_SCHEDULER_H
