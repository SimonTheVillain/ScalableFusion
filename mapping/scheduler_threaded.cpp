#include "scheduler_threaded.h"

#include <fstream>
#include <thread>
//mainly for naming threads
#include <pthread.h>
#include <unistd.h>

#include <mesh_reconstruction.h>
#include <intermediate_depth_model.h>
#include <dataset_loader/dataset_loader.h>
#include <icp_cuda/icp_odometry.h>
#include <gpu/active_set.h>
#include <cuda/xtion_camera_model.h>
#include <gfx/garbage_collector.h>
#include <worker.h>

using namespace std;
using namespace Eigen;

void SchedulerThreaded::captureWorker_(shared_ptr<MeshReconstruction> map, 
                                       Stream *stream, GLFWwindow *context) {
/*
	//stuff that formerly has been in capture proc:
	//**********************************
	GLFWwindow *new_context = createConnectedGlContext(context);
	glfwMakeContextCurrent(new_context);

	//this has to be done once for every gl context or thread:
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();
	//*********************************

	ofstream logMemory("/home/simon/Desktop/scalableLog.txt");

	float depth_cutoff = 4.0f;
	int threads = 256;
	int blocks = 16;

	map->initInGlLogicContext();



	InformationRenderer information_renderer;

	information_renderer.initInContext();

	information_renderer.initInContext(
			640,// TODO: don't hardcode this!!!
			480,
			map.get());

	Vector4f fc = stream->getDepthIntrinsics();

	//Camera tracking via cuda icp
	ICPOdometry *odometry = new ICPOdometry(640, 480, fc[2], fc[3], fc[0], fc[1]);

	IntermediateMap intermediate_map(640, 480, fc);

	map->setDepthIntrinsics(stream->getDepthIntrinsics());
	map->setRGBIntrinsics(stream->getRgbIntrinsics());

	//this is a replacement for a proper test if there already is something added to the map
	bool first_lap = true;

	Sophus::SE3d accu_pose;

	accu_pose.setRotationMatrix(Matrix3d::Identity());
	accu_pose.translation() = Vector3d(0, 0, 0);

	Matrix4f rel_depth_to_color = stream->getDepth2RgbRegistration();

	//as soon as we have a frame we create
	while(stream->isRunning() && !end_threads_) {
		stream->readNewSetOfImages();
		if(!stream->isRunning()) {
			break; // end this loop if there is no new image
			//This could be written more efficiently (but who cares about beautiful code?)
		}
		cout << "[SchedulerThreaded::captureWorker] pulling another frame from the dataset/stream" << endl;

		cv::Mat  depth = stream->getDepthFrame(); // 16 bit 1mm resolution
		cv::Mat  rgb   = stream->getRgbFrame(); // 8 bit 3 channels (usually)
		Matrix4f pose  = stream->getDepthPose();

		if(depth.type() != CV_16UC1) {
			assert(0);
		}

		cv::Mat depthu16;
		depth.convertTo(depthu16, CV_16UC1);//mm resolution needed needed vor ICPCUDA

		odometry->initICP((unsigned short*) depthu16.data, depth_cutoff);

		if(first_lap) {
			odometry->initICPModel((unsigned short*) depthu16.data, depth_cutoff);
			intermediate_map.setDepthMap(depthu16, accu_pose.cast<float>().matrix());
		}

		map->active_set_update_mutex.lock();
		shared_ptr<ActiveSet> active_set_capturing = map->active_set_update;
		map->active_set_update_mutex.unlock();

		bool has_geometry = false;
		if(map->hasGeometry() && active_set_capturing != nullptr) {
			if(active_set_capturing->retained_mesh_patches.size() !=0 ) {
				has_geometry = true;
			}
		}
		if(has_geometry) {
			//update the odometry when we already have geometry mapped
			cv::Mat reprojected_depth =
					map->generateDepthFromView(640, 480,information_renderer, accu_pose.cast<float>().matrix());

			odometry->initICPModel((unsigned short*) reprojected_depth.data,
			                       depth_cutoff);

			//previous: apply as long as there is no geometry
			Sophus::SE3d rel_pose;
			odometry->getIncrementalTransformation(rel_pose, threads,blocks);
			accu_pose = accu_pose * rel_pose;

		} else {
			//if there is no geometry in the reconstruction yet, we perform tracking relative to the reference frame
			Sophus::SE3d rel_pose;
			odometry->getIncrementalTransformation(rel_pose, threads, blocks);

			cv::Mat rerender =
					intermediate_map.renderDepthMap(accu_pose.cast<float>().matrix());
			odometry->initICPModel((unsigned short*) rerender.data, depth_cutoff);
		}
		odometry_timer_.click();

		Matrix4f depth_pose = accu_pose.cast<float>().matrix();
		Matrix4f rgb_pose = rel_depth_to_color * depth_pose;

		if(stream->hasGroundTruth()) {
			depth_pose = stream->getDepthPose();
			rgb_pose   = stream->getRgbPose();
		}
		last_known_depth_pose_ = depth_pose;

		//upload the data to the gpu
		cv::Mat rgba;
		cv::cvtColor(rgb, rgba, cv::COLOR_BGR2RGBA);

		shared_ptr<gfx::GpuTex2D> rgb_texture = 
				make_shared<gfx::GpuTex2D>(garbage_collector_, GL_RGBA, GL_RGBA,
				                           GL_UNSIGNED_BYTE, rgba.cols, rgba.rows, 
				                           true, rgba.data);

		//************************************************
		//Prepare the sensor data

		//do the same for depth!
		cv::Mat depthf;
		//the depth is in mm
		depth.convertTo(depthf, CV_32FC1, 1.0f / 1000.0f);
		//create the depth map and also the depth standardDeviation on the gpu. Further elements will use this
		shared_ptr<gfx::GpuTex2D> depth_tex =
				make_shared<gfx::GpuTex2D>(garbage_collector_, GL_R32F, GL_RED, 
				                           GL_FLOAT, depth.cols, depth.rows, true,
				                           static_cast<void*>(depthf.data));
		//upload the data to the depth tex.
		shared_ptr<gfx::GpuTex2D> d_std_tex =
				make_shared<gfx::GpuTex2D>(garbage_collector_, GL_RGBA32F, GL_RGBA,
				                           GL_FLOAT, depth.cols, depth.rows, true,
				                           nullptr);

		//create the standard deviation Information:
		generateXtionConfidenceImage(depth_tex->getCudaSurfaceObject(),
		                             d_std_tex->getCudaSurfaceObject(),
		                             depth.cols,depth.rows);

		cv::Mat d_std_mat(depth.rows, depth.cols, CV_32FC4);
		d_std_tex->downloadData(static_cast<void*>(d_std_mat.data));

		//***************************************
		//lets integrate the new data
		//get current active set:
		map->active_set_update_mutex.lock();
		shared_ptr<ActiveSet> active_set = map->active_set_update;
		map->active_set_update_mutex.unlock();

		//refine rgb and depth data on patches residing in the current active set
		auto refine_depth = bind(&SchedulerThreaded::refineDepth_, this, 
		                         active_set, d_std_tex, depth_pose);
		refine_depth_worker_->setNextTask(refine_depth);

		//update the active set as often as possible:
		//once in a while the active set is updated while the worker for the geometry integration
		//is not busy. in this case the task in the worker is starting a new process
		//also, whenever we manage the download of geometry from here
		//and the start of a new expand process.
		auto update_active_set = bind(&SchedulerThreaded::updateActiveSet_, this,
		                              d_std_mat,d_std_tex, depth_pose, rgb_texture,
		                              rgb_pose);
		update_active_set_worker_->setNextTask(update_active_set);

		//debug: printing the number of used vertices into a file
		logMemory << map_->gpu_geom_storage_.vertex_buffer->getUsedElements() << endl;

		first_lap = false;

		//TODO: remove this
		//this down here is the old single threaded pipeline:
		while(!frame_done_debug_synchronize_ && false) { //actially don't wait for this
			//this is not the best option
			usleep(10000000);
			//this is waaay better:
			//this_thread::sleep_for
		}
		frame_done_debug_synchronize_ = false;
	}// end of the whole capturing process

	cout << "No new Frames: Ending capture thread!" << endl;
	delete odometry;

	//TODO: all the cleanup!!!!

	//Detroying the context which is only used in this thread
	glfwDestroyWindow(new_context);

*/
}

void SchedulerThreaded::updateActiveSet_(cv::Mat d_std_mat, 
                                         shared_ptr<gfx::GpuTex2D> d_std_tex,
                                         Matrix4f depth_pose, 
                                         shared_ptr<gfx::GpuTex2D> rgb_tex,
                                         Matrix4f rgb_pose) {
	/*
	bool do_expansion_update = false;
	if(!currently_working_on_expansion_) {
		currently_working_on_expansion_ = true;
		do_expansion_update = true;
	}

	shared_ptr<ActiveSet> active_set = map_->genActiveSetFromPose(depth_pose,);

	map_->setActiveSetUpdate_(active_set);

	//debug... check if the active set has all the geometry textures
	for(shared_ptr<MeshPatch> patch : active_set->retained_mesh_patches_cpu) {
		//TODO: test patch
		if(!patch->isPartOfActiveSetWithNeighbours(active_set.get())) {
			continue;
		}
		if(patch->geom_tex_patch->gpu.lock() == nullptr) {
			cout << "[Scheduler::updateActiveSet] DEBUG/TODO: reinsert the assert at this point" << endl;

			//assert(0);//whyyyyy. the geometry textures should be secured at
			//this point
		}
	}

	update_active_set_timer_.click();

	//if we are threaded and the last expand method is finished
	//we start a new expansion process

	if(do_expansion_update) { //but only if the last expansion was finished before
		//starting this expansion (otherwise we risk doubling our geometry)
		auto expand = bind(&SchedulerThreaded::expand_, this, active_set, rgb_tex,
		                   rgb_pose, d_std_tex, d_std_mat, depth_pose);
		expand_worker_->setNextTask(expand);
	}
	 */
}

void SchedulerThreaded::refineRgb_(shared_ptr<ActiveSet> active_set, 
                                   shared_ptr<gfx::GpuTex2D> rgb_tex,
                                   Matrix4f rgb_pose) {
	//map_->texturing.colorTexUpdate(rgb_tex, rgb_pose, active_set);
	refine_rgb_timer_.click();
}

void SchedulerThreaded::refineDepth_(shared_ptr<ActiveSet> active_set, 
                                     shared_ptr<gfx::GpuTex2D> d_std_tex,
                                     Matrix4f depth_pose) {
	//map_->geometry_update.update(d_std_tex, depth_pose, active_set);
	refine_depth_timer_.click();
}

void SchedulerThreaded::expand_(shared_ptr<ActiveSet> active_set, 
                                shared_ptr<gfx::GpuTex2D> rgb_tex,
                                Matrix4f rgb_pose, 
                                shared_ptr<gfx::GpuTex2D> d_std_tex, 
                                cv::Mat d_std_mat, Matrix4f depth_pose) {
	//map_->geometry_update.extend(active_set, d_std_tex,d_std_mat, depth_pose,
	//                            rgb_tex, rgb_pose);
	currently_working_on_expansion_ = false;
	frame_done_debug_synchronize_ = true;
	expand_timer_.click();
}

SchedulerThreaded::SchedulerThreaded(shared_ptr<MeshReconstruction> map, 
                                     Stream *stream, GLFWwindow *context)
		: last_known_depth_pose_(Matrix4f::Identity()),
		  map_(map),
		  step_trough_(true),
		  end_threads_(false),
		  currently_working_on_expansion_(false),
		  expand_interval_(10),// this means every 10 frames, the map gets expanded 
		                       // (only when we are not in the threaded mode)
		  frame_count_(10),
		  frame_done_debug_synchronize_(false) {

	//return;//TODO: remove this desparate debug measure

	auto initializerLogic = [&](GLFWwindow *parent_context, 
	                            GLFWwindow **personal_context) {
		*personal_context = createConnectedGlContext(parent_context);
		initializeGlContextInThread(*personal_context);
		map->initInGlLogicContext();
	};

	auto initializer = [&](GLFWwindow *parent_context, 
	                       GLFWwindow **personal_context) {
		*personal_context = createConnectedGlContext(parent_context);
		initializeGlContextInThread(*personal_context);
	};

	auto cleaner = [&map](GLFWwindow **personal_context) {
		map->fbo_storage_.forceGarbageCollect();
		map->garbage_collector_->forceCollect();
		glFinish();
		glfwDestroyWindow(*personal_context);
		delete personal_context;
	};

	//TODO: i think there was something about getting the creation and deletion of active sets into the same thread...?
	{
		GLFWwindow **context_for_worker = new GLFWwindow*;
		expand_worker_ = new Worker(bind(initializerLogic, context, 
		                                 context_for_worker), "expand",
		                            bind(cleaner, context_for_worker));
	}

	//for creating an active set and filling up the
	{
		GLFWwindow **context_for_worker = new GLFWwindow*;
		update_active_set_worker_ = new Worker(bind(initializer, context, 
		                                            context_for_worker), 
		                                       "updateSet",
		                                       bind(cleaner, context_for_worker));
	}

	{
		GLFWwindow **context_for_worker = new GLFWwindow*;
		refine_rgb_worker_ = new Worker(bind(initializer, context, 
		                                     context_for_worker), "refineRgb",
		                                bind(cleaner, context_for_worker));
	}

	{
		GLFWwindow **context_for_worker = new GLFWwindow*;
		refine_depth_worker_ = new Worker(bind(initializer, context, 
		                                       context_for_worker), "refineDepth",
		                                  bind(cleaner, context_for_worker));
	}

	cout << "[Scheduler::Scheduler] TODO: delete these contexts when necessary" << endl;

	//if we are running non multithreaded this is the only thing thats running
	capture_thread_ = thread(&SchedulerThreaded::captureWorker_, this, map, 
	                         stream, context);

	pthread_setname_np(capture_thread_.native_handle(), "capture");

	odometry_timer_.name          = "odometry";
	update_active_set_timer_.name = "ActiveSet";
	refine_rgb_timer_.name        = "refineRgb";
	refine_depth_timer_.name      = "refineDepth";
	expand_timer_.name            = "expand";

	#ifndef LOG_FRAMERATES
	odometry_timer_.mute          = true;
	update_active_set_timer_.mute = true;
	refine_rgb_timer_.mute        = true;
	refine_depth_timer_.mute      = true;
	expand_timer_.mute            = true;
	#endif
}

SchedulerThreaded::~SchedulerThreaded() {
	//first send message to threads so they stop doing work
	end_threads_ = true;
	capture_thread_.join();
	delete expand_worker_;
	delete refine_rgb_worker_;
	delete refine_depth_worker_;
	delete update_active_set_worker_;
}