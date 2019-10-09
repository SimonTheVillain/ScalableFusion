#include "scheduler.h"

#include <iostream>
#include <thread>
#include <pthread.h>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../icpCUDA/ICPOdometry.h"
//TODO: remove since it is not needed in this class
#include "intermediateDepthModel.h"
#include "../segmentation/IncrementalSegmentation.h"
#include "cuda/xtionCameraModel.h"
#include "../datasetLoader/datasetLoader.h"
#include <GarbageCollector.h>
#include "meshReconstruction.h"
#include "../gpu/ActiveSet.h"

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
		shared_ptr<IncrementalSegmentation> incremental_segmentation)
		: incremental_segmentation_(incremental_segmentation),
		  last_known_depth_pose_(Matrix4f::Identity()),
		  garbage_collector_(garbage_collector),
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

void SchedulerLinear::captureWorker_(shared_ptr<MeshReconstruction> map, 
                                    Stream *stream, GLFWwindow *context) {
	//TODO: Check: creating a connected context might invalidate our efforts to properly destroy all of this threads resources

	GLFWwindow* connected_context = createConnectedGlContext(context);
	glfwMakeContextCurrent(connected_context);
	//this has to be done once for every gl context or thread:
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();

	float depth_cutoff = 4.0f;
	int threads = 256;
	int blocks = 16;

	int frame_count = expand_interval_;

	map->initInGlLogicContext();

	incremental_segmentation_->initInThread();

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

	shared_ptr<ActiveSet> active_set_last_expand;
	Matrix4f depth_pose_last_expand;

	while(stream->isRunning() && !end_threads_) {
		if(paused_ && !take_next_step_){
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

		odometry->initICP((unsigned short*)depthu16.data, depth_cutoff);


		if(first_lap) {
			odometry->initICPModel((unsigned short*)depthu16.data, depth_cutoff);

		} else {
			cv::Mat reprojected_depth = 
					map->generateDepthFromView(640, 480, accu_pose.cast<float>().matrix());

			odometry->initICPModel((unsigned short*) reprojected_depth.data, depth_cutoff);

			Sophus::SE3d rel_pose;
			odometry->getIncrementalTransformation(rel_pose, threads, blocks);
			accu_pose = accu_pose * rel_pose;
		}

		map->active_set_update_mutex.lock();
		shared_ptr<ActiveSet> active_set_capturing = map->active_set_update;
		map->active_set_update_mutex.unlock();

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
		shared_ptr<ActiveSet> active_set = map->genActiveSetFromPose(depth_pose);
		//map->debugCheckTriangleNeighbourConsistency(map->GetAllPatches());

		//don't ask me what this is doing here!TODO: find out
		map->clearInvalidGeometry(active_set, depth, depth_pose);

		map->geometry_update.update(d_std_tex, depth_pose, active_set);

		map->texturing.colorTexUpdate(rgb_texture, rgb_pose, active_set);

		//there definitely is a reason to keep the active set here!
		map->setActiveSetUpdate_(active_set);

		//every now and then we add new geometry:
		if(frame_count == expand_interval_ || first_lap) {

			/************* SEMANTIC LABELLING ***********************/
			if (!first_lap && false) {

				//TODO: we actually always want to use the pose and active_set from the last expand step
				//THIS IS VERY VALID!!!!!!!!!!!!!!!!!!!!!!!!!!!

				//First we need to render the labels in all possible constellations
				cv::Mat reprojected_depth = 
						map->generateDepthFromView(640, 480, depth_pose_last_expand);
				imshow("reprojected depth", reprojected_depth);

				//render all the necessary info needed for labelling
				Matrix4f proj = map->genDepthProjMat();
				cv::Mat rendered_depth(depth.rows, depth.cols, CV_32FC1);
				cv::Mat rendered_normals(depth.rows, depth.cols, CV_32FC4);
				cv::Mat rendered_labels(depth.rows, depth.cols, CV_32FC4);//SC1?
				cv::Mat rendered_color(depth.rows, depth.cols, CV_32FC4);
				map->information_renderer.render(active_set.get(),//activeSetExpand
				                                 proj, depth_pose_last_expand,
				                                 &rendered_depth, &rendered_normals, 
				                                 &rendered_color, &rendered_labels);

				cv::Mat novellabels =
						incremental_segmentation_->generateNewLabels(&rendered_depth,
						                                             &rendered_normals,
						                                             &rendered_color,
						                                             &rendered_labels);

				//then we run the labelling
				imshow("rendered_depth", rendered_depth * 0.25f);
				imshow("rendered_normals", rendered_normals);
				imshow("rendered_labels", rendered_labels);
				imshow("rendered_color", rendered_color);
				cout << rendered_labels.at<cv::Vec4i>(100, 100) << endl;

				cv::waitKey();

				//now we project the labels
				//fake labels, best labes
				cv::Mat new_labels(depth.rows, depth.cols, CV_32SC4);
				new_labels.setTo(cv::Scalar(100000, 1, 1));
				cv::imshow("new_labels", new_labels);
				//maybe instead of the dStdTex we use the new label texture
				map->labelling.projectLabels(active_set, new_labels, d_std_tex, 
				                             depth_pose_last_expand);
			}
			/*********************************************************/


			//expanding the existing geometry
			map->geometry_update.extend(active_set, d_std_tex, d_std_mat, depth_pose,
			                            rgb_texture, rgb_pose);

			//setting the active set, which also gets rendered to
			//the one updated in the expand method.
			//only do this in single threaded mode.
			map->setActiveSetUpdate_(map->active_set_expand);

			frame_count = 0;

			active_set_last_expand = map->active_set_expand;
			depth_pose_last_expand = depth_pose;
		}

		frame_count++;

		//debug: printing the number of used vertices into a file

		first_lap = false;

		//cleanup VBO and VAOs that are deleted but only used within this thread
		map->cleanupGlStoragesThisThread_();
		garbage_collector_->collect();

		cv::imshow("rgb", rgb);
		cv::waitKey(1);
		//tons of debug output to find this fucking memory leak!!!!
		int tex_count = map->tex_atlas_stds_->countTex() +
		                map->tex_atlas_geom_lookup_->countTex() +
		                map->tex_atlas_rgb_8_bit_->countTex() +
		                map->tex_atlas_seg_labels_->countTex();
		cout << "texCount overall: " << tex_count << " stds " << 
		        map->tex_atlas_stds_->countTex() << " lookup " <<
		        map->tex_atlas_geom_lookup_->countTex() << " rgb " << 
		        map->tex_atlas_rgb_8_bit_->countTex() << endl;

		int patch_count = map->tex_atlas_stds_->countPatches() +
		                  map->tex_atlas_geom_lookup_->countPatches() +
		                  map->tex_atlas_rgb_8_bit_->countPatches() +
		                  map->tex_atlas_seg_labels_->countPatches();

		cout << "patchCount overall: " << patch_count << " stds " << 
		        map->tex_atlas_stds_->countPatches() << " lookup " <<
		        map->tex_atlas_geom_lookup_->countPatches() << " rgb " << 
		        map->tex_atlas_rgb_8_bit_->countPatches() << endl;

		cout << "FBOs active " << map->getFboCountDebug_() << endl;
	}

	//delete everything for the fbo
	map->fbo_storage_.forceGarbageCollect();
	garbage_collector_->forceCollect();
	glFinish();

	glfwDestroyWindow(connected_context);
	delete odometry;

	cout << "DEBUG: scheduler finished thread" <<endl;
}