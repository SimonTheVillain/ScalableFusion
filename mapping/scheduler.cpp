#include "scheduler.h"

#include <iostream>
#include <thread>
#include <pthread.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <icp_cuda/icp_odometry.h>
//TODO: remove since it is not needed in this class
#include <intermediate_depth_model.h>
#include <segmentation/incremental_segmentation.h>
#include <cuda/xtion_camera_model.h>
#include <dataset_loader/dataset_loader.h>
#include <gfx/garbage_collector.h>
#include <mesh_reconstruction.h>
#include <gpu/active_set.h>

using namespace std;
using namespace Eigen;

GLFWwindow *SchedulerBase::createConnectedGlContext(GLFWwindow *context) {
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_VISIBLE, 0);
	return glfwCreateWindow(640, 480, "HOPE U NO VISIBLE", nullptr, context);
}

void SchedulerBase::initializeGlContextInThread(GLFWwindow *context) {
	glfwMakeContextCurrent(context);

	//this has to be done once for every gl context or thread:
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();
}

SchedulerLinear::SchedulerLinear(
		shared_ptr<MeshReconstruction> map,  GarbageCollector *garbage_collector,
		Stream *stream, GLFWwindow *context,
		LowDetailRenderer* low_detail_renderer,
		shared_ptr<IncrementalSegmentation> incremental_segmentation)
		: incremental_segmentation_(incremental_segmentation),
		  last_known_depth_pose_(Matrix4f::Identity()),
		  garbage_collector_(garbage_collector),
		  low_detail_renderer_(low_detail_renderer),
		  expand_interval_(30),
		  end_threads_(false),
		  paused_(false),
		  take_next_step_(false) {

	capture_thread_ = thread(&SchedulerLinear::captureWorker_, this, map, stream, context);
	pthread_setname_np(capture_thread_.native_handle(),"capture");
}

SchedulerLinear::~SchedulerLinear() {
	end_threads_ = true;
	capture_thread_.join();
	cout << "DEBUG: scheduler destroyed" << endl;
}

void SchedulerLinear::captureWorker_(shared_ptr<MeshReconstruction> reconstruction,
                                     Stream *stream, GLFWwindow *context) {
	//TODO: Check: creating a connected context might invalidate our efforts to properly destroy all of this threads resources

	GLFWwindow *connected_context = createConnectedGlContext(context);
	glfwMakeContextCurrent(connected_context);
	//this has to be done once for every gl context or thread:
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();

	float depth_cutoff = 4.0f;
	int threads = 256;
	int blocks = 16;

	int frame_count = expand_interval_;

	reconstruction->initInGlLogicContext();

	InformationRenderer* information_renderer =
		new InformationRenderer();

	information_renderer->initInContext(reconstruction.get());

	information_renderer->initInContext(
			640,// TODO: don't hardcode this!!!
			480,
			reconstruction.get());

	GeometryUpdater* geometry_updater =
			new GeometryUpdater();
	TextureUpdater* texture_updater =
			new TextureUpdater();
	//texture_updater.mesh_reconstruction = map.get();
	geometry_updater->setup(reconstruction.get());



	incremental_segmentation_->initInThread();

	Vector4f fc = stream->getDepthIntrinsics();

	//Camera tracking via cuda icp
	ICPOdometry *odometry = new ICPOdometry(640, 480, fc[2], fc[3], fc[0], fc[1]);

	IntermediateMap intermediate_map(640, 480, fc);

	reconstruction->setDepthIntrinsics(stream->getDepthIntrinsics());
	reconstruction->setRGBIntrinsics(stream->getRgbIntrinsics());

	//this is a replacement for a proper test if there already is something added to the map
	bool first_lap = true;

	Sophus::SE3d accu_pose;

	accu_pose.setRotationMatrix(Matrix3d::Identity());
	accu_pose.translation() = Vector3d(0, 0, 0);

	Matrix4f rel_depth_to_color = stream->getDepth2RgbRegistration();

	shared_ptr<ActiveSet> active_set_last_expand;
	Matrix4f depth_pose_last_expand;

	while(stream->isRunning() && !end_threads_) {
		if(paused_ && !take_next_step_) {
			continue;
		}
		take_next_step_ = false;
		stream->readNewSetOfImages();
		if(!stream->isRunning()) {
			break; // end this loop if there is no new image
			//This could be written more efficiently (but who cares about beautiful code?)
		}

		cv::Mat depth = stream->getDepthFrame(); // 16 bit 1mm resolution
		cv::Mat rgb   = stream->getRgbFrame(); // 8 bit 3 channels (usually)
		Matrix4f pose = stream->getDepthPose();

		if(depth.type() != CV_16UC1) {
			assert(0);
		}

		cv::Mat depthu16;
		depth.convertTo(depthu16, CV_16UC1);//mm resolution needed needed vor ICPCUDA

		odometry->initICP((unsigned short*) depthu16.data, depth_cutoff);

		if(first_lap) {
			odometry->initICPModel((unsigned short*) depthu16.data, depth_cutoff);

		} else {
			cv::Mat reprojected_depth = 
					reconstruction->generateDepthFromView(640, 480, information_renderer, accu_pose.cast<float>().matrix());

			odometry->initICPModel((unsigned short*) reprojected_depth.data, depth_cutoff);

			Sophus::SE3d rel_pose;
			odometry->getIncrementalTransformation(rel_pose, threads, blocks);
			accu_pose = accu_pose * rel_pose;
		}

		reconstruction->active_set_update_mutex.lock();
		shared_ptr<ActiveSet> active_set_capturing = reconstruction->active_set_update;
		reconstruction->active_set_update_mutex.unlock();

		Matrix4f depth_pose = accu_pose.cast<float>().matrix();
		Matrix4f rgb_pose   = rel_depth_to_color * depth_pose;

		//If there is groundtruth trajectory loaded, we will use it!!!
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
		rgb_texture->name = "[scheduler] rgb_texture";

		//do the same for depth!
		cv::Mat depthf;
		//the depth is in mm
		depth.convertTo(depthf, CV_32FC1, 1.0f / 1000.0f);
		//create the depth map and also the depth standardDeviation on the gpu. Further elements will use this
		shared_ptr<gfx::GpuTex2D> depth_tex = 
				make_shared<gfx::GpuTex2D>(garbage_collector_, GL_R32F, GL_RED, GL_FLOAT,
				                           depth.cols, depth.rows, true, 
				                           static_cast<void*>(depthf.data));
		depth_tex->name = "[scheduler] depth texture";

		//upload the data to the depth tex.
		shared_ptr<gfx::GpuTex2D> d_std_tex = 
				make_shared<gfx::GpuTex2D>(garbage_collector_, GL_RGBA32F, GL_RGBA, 
				                           GL_FLOAT, depth.cols, depth.rows, true,
				                           nullptr);
		d_std_tex->name = "[scheduler] d_std_tex";

		//create the standard deviation Information:
		generateXtionConfidenceImage(depth_tex->getCudaSurfaceObject(), 
		                             d_std_tex->getCudaSurfaceObject(),
		                             depth.cols, depth.rows);

		cv::Mat d_std_mat(depth.rows, depth.cols, CV_32FC4);
		d_std_tex->downloadData(static_cast<void*>(d_std_mat.data));

		/*********************************************************************/
		//now integrate everything new:
		//Update the active set according to the current pose:
		//map->debugCheckTriangleNeighbourConsistency(map->GetAllPatches());
		shared_ptr<ActiveSet> active_set =
				reconstruction->genActiveSetFromPose(depth_pose,
													 low_detail_renderer_, //that low detail renderer needs to be shared with
						                             texture_updater,
													 information_renderer);
		//map->debugCheckTriangleNeighbourConsistency(map->GetAllPatches());

		//don't ask me what this is doing here!TODO: find out
		reconstruction->clearInvalidGeometry(active_set, depth, depth_pose);

		geometry_updater->update(d_std_tex, depth_pose, active_set);

		texture_updater->colorTexUpdate(reconstruction.get(),rgb_texture,low_detail_renderer_, rgb_pose, active_set);

		//there definitely is a reason to keep the active set here!
		reconstruction->setActiveSetUpdate_(active_set);

		//every now and then we add new geometry:
		if(frame_count == expand_interval_ || first_lap) {

			/************* SEMANTIC LABELLING ***********************/

			/*********************************************************/

			//expanding the existing geometry
			geometry_updater->extend(
					reconstruction.get(), information_renderer, texture_updater, low_detail_renderer_,
					active_set, d_std_tex, d_std_mat, depth_pose,
					rgb_texture, rgb_pose);

			//setting the active set, which also gets rendered to
			//the one updated in the expand method.
			//only do this in single threaded mode.
			reconstruction->setActiveSetUpdate_(reconstruction->active_set_expand);

			frame_count = 0;

			active_set_last_expand = reconstruction->active_set_expand;
			depth_pose_last_expand = depth_pose;
		}

		frame_count++;

		//debug: printing the number of used vertices into a file

		first_lap = false;

		//cleanup VBO and VAOs that are deleted but only used within this thread
		reconstruction->cleanupGlStoragesThisThread_();
		garbage_collector_->collect();

		cv::imshow("rgb", rgb);
		cv::waitKey(1);
		//tons of debug output to find this fucking memory leak!!!!
		int tex_count = reconstruction->tex_atlas_stds_->countTex() +
						reconstruction->tex_atlas_geom_lookup_->countTex() +
						reconstruction->tex_atlas_rgb_8_bit_->countTex() +
						reconstruction->tex_atlas_seg_labels_->countTex();
		cout << "texCount overall: " << tex_count << " stds " <<
			 reconstruction->tex_atlas_stds_->countTex() << " lookup " <<
			 reconstruction->tex_atlas_geom_lookup_->countTex() << " rgb " <<
			 reconstruction->tex_atlas_rgb_8_bit_->countTex() << endl;

		int patch_count = reconstruction->tex_atlas_stds_->countPatches() +
						  reconstruction->tex_atlas_geom_lookup_->countPatches() +
						  reconstruction->tex_atlas_rgb_8_bit_->countPatches() +
						  reconstruction->tex_atlas_seg_labels_->countPatches();

		cout << "patchCount overall: " << patch_count << " stds " <<
			 reconstruction->tex_atlas_stds_->countPatches() << " lookup " <<
			 reconstruction->tex_atlas_geom_lookup_->countPatches() << " rgb " <<
			 reconstruction->tex_atlas_rgb_8_bit_->countPatches() << endl;

		cout << "FBOs active " << reconstruction->getFboCountDebug_() << endl;
	}

	delete information_renderer;
	delete geometry_updater;
	delete texture_updater;
	//delete everything for the fbo
	reconstruction->fbo_storage_.forceGarbageCollect();
	garbage_collector_->forceCollect();
	glFinish();



	glfwDestroyWindow(connected_context);
	delete odometry;

	cout << "DEBUG: scheduler finished thread" <<endl;
}