#ifndef FILE_SCHEDULER_THREADED_H
#define FILE_SCHEDULER_THREADED_H

#include "scheduler.h"

using namespace std;
using namespace Eigen;

class SchedulerThreaded : public SchedulerBase {
public:

	SchedulerThreaded(shared_ptr<MeshReconstruction> map,
	                  Stream *stream, GLFWwindow *context);

	~SchedulerThreaded();

	void pause(bool pause) {
		assert(0); //unimplemented
	}

	void nextStep() {
		assert(0); // unimplemented
	}

	Matrix4f getLastKnownDepthPose() {
		return last_known_depth_pose_;
	}

private:

	//TODO: these three:
	/*
	 * The active set is coming from a method which just loads the visible patches
	 * which stores the active set into the activeSetReconstruction variable.
	 * The update methods read out the active set from exactly this capturing active set.
	 * The second method storing to this active set is the expand method,
	 * the update and expand method have to make sure not to overwrite their contents
	 * if they do not want to.
	 */
	void updateActiveSet_(cv::Mat d_std_mat, shared_ptr<gfx::GpuTex2D> d_std_tex, 
	                      Matrix4f depth_pose, shared_ptr<gfx::GpuTex2D> rgb_tex, 
	                      Matrix4f rgb_pose);

	void refineRgb_(shared_ptr<ActiveSet> active_set, 
	                shared_ptr<gfx::GpuTex2D> rgb_tex,
	                Matrix4f rgb_pose);

	void refineDepth_(shared_ptr<ActiveSet> active_set,
	                  shared_ptr<gfx::GpuTex2D> d_std_tex, 
	                  Matrix4f depth_pose);


	void expand_(shared_ptr<ActiveSet> active_set, 
	             shared_ptr<gfx::GpuTex2D> rgb_tex, Matrix4f rgb_pose,
	             shared_ptr<gfx::GpuTex2D> d_std_tex, cv::Mat d_std_mat, 
	             Matrix4f depth_pose);

	void segmentLabel_(shared_ptr<ActiveSet> active_set, cv::Mat rgb, 
	                   cv::Mat depth, shared_ptr<gfx::GpuTex2D> d_std_tex,
	                   Matrix4f pose);

	void captureWorker_(shared_ptr<MeshReconstruction> map, Stream *stream, 
	                    GLFWwindow *context);

	//TODO: this
	GarbageCollector *garbage_collector_;
	//bool usingGroundtruth;
	bool step_trough_;
	//bool threaded; //This is always threaded!!!!!!!!
	bool end_threads_;

	bool currently_working_on_expansion_;

	int expand_interval_;
	int frame_count_;

	shared_ptr<MeshReconstruction> map_;

	Segmentation *attention_segmenter_;

	thread capture_thread_;

	//TODO: these three
	Worker *update_active_set_worker_;
	Worker *refine_rgb_worker_;
	Worker *refine_depth_worker_;

	Worker *expand_worker_;

	FPSCounter odometry_timer_;
	FPSCounter update_active_set_timer_;
	FPSCounter refine_rgb_timer_;
	FPSCounter refine_depth_timer_;

	FPSCounter expand_timer_;

	Worker *upload_worker_;//TODO: creates the active set
	// ideally the upload process is taking place while the textures
	//and the geometry is getting refined so that they can work on the patches
	//visible in the last frame, and the expansion worker can work on the new textures
	//PROBLEMATIC: Uploading textures from the cpu that got newer(better) gpu counterparts
	//at the same moment.

	//There is a separate thread for downloading stuff in the mapping
	//class itself. This might cause some fragmentation in the code:
	//TODO: unify the scheduler and the map class. (somehow)
	//maybe we even just give the ownership of the scheduler to map class.

	bool frame_done_debug_synchronize_;

	//actually we should secure this via a mutex
	Matrix4f last_known_depth_pose_;
};



#endif
