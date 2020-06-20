#ifndef FILE_SCHEDULER_H
#define FILE_SCHEDULER_H

#include <thread>
#include <mutex>

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



	//this design allows for only one render cam but if there will be more needed we cross that bridge another time
	virtual void setRenderCamPose(Matrix4f camPose) = 0;
	virtual shared_ptr<ActiveSet> getActiveSetRendering() = 0;


	//TODO: these:!! (at least partially)
	/*
	virtual shared_ptr<ActiveSet> getActiveSetExtension() = 0;
	virtual shared_ptr<ActiveSet> setActiveSetExtension(shared_ptr<ActiveSet> set) = 0;

	virtual shared_ptr<ActiveSet> getActiveSetGeomUpdate() = 0;
	virtual shared_ptr<ActiveSet> setActiveSetGeomUpdate(shared_ptr<ActiveSet> set) = 0;


	virtual shared_ptr<ActiveSet> getActiveSetTexUpdate() = 0;
	virtual shared_ptr<ActiveSet> setActiveSetTexUpdate(shared_ptr<ActiveSet> set) = 0;
	 */

	//is handling active sets the responsibility of the scheduler?
	virtual vector<shared_ptr<ActiveSet>> getActiveSets() = 0;

	//we probably also want to update the low detail renderer whenever we update the render cam pose
	//with update i mean recreate and replace
	virtual shared_ptr<LowDetailRenderer> getLowDetailRenderer() = 0;

};

class SchedulerLinear : public SchedulerBase {
private:
    bool rendering_sets_updated[2] = {false, false};
    int rendering_sets_index = 0;
    int skip_interval_ = 1;
public:

	SchedulerLinear(shared_ptr<MeshReconstruction> map,
					GpuStorage* gpu_storage, video::Source *source,
	                GLFWwindow *context,
	                LowDetailRenderer *low_detail_renderer,
	                int skip_interval);

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

	virtual void setRenderCamPose(Matrix4f camPose){

	}
	//TODO: make this private:
	void setActiveSetRendering(shared_ptr<ActiveSet> set){
	    active_sets_mutex.lock();
        int other_ind = rendering_sets_index + 1;
        if(other_ind > 1)
            other_ind = 0;
        active_sets[1 + other_ind] = set;
        rendering_sets_updated[other_ind] = true;
	    active_sets_mutex.unlock();
	}
	virtual shared_ptr<ActiveSet> getActiveSetRendering(){
		active_sets_mutex.lock();
		int other_ind = rendering_sets_index + 1;
		if(other_ind > 1)
		    other_ind = 0;

		//if there already is an updated version of the other
		if(rendering_sets_updated[other_ind]){
		    rendering_sets_index = other_ind;
		    rendering_sets_updated[other_ind] = false;
		}
		rendering_sets_updated[rendering_sets_index] = false;
		auto set = active_sets[1 + rendering_sets_index];
		active_sets_mutex.unlock();
		return set;
	}

	//we probably also want to update the low detail renderer whenever we update the render cam pose
	//with update i mean recreate and replace
	virtual shared_ptr<LowDetailRenderer> getLowDetailRenderer(){
		return nullptr;
	}

	vector<shared_ptr<ActiveSet>> getActiveSets();

private:
	//the active sets used for several operations like rendering and update
	mutex active_sets_mutex;
	vector<shared_ptr<ActiveSet>> active_sets;


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
