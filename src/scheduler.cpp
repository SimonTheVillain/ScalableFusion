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
#include <cuda/xtion_camera_model.h>
#include <video_source/include/source.h>
#include <gpu/garbage_collector.h>
#include <mesh_reconstruction.h>
#include <gpu/active_set.h>
#include <gpu/camera.h>

using namespace std;
using namespace Eigen;

GLFWwindow *SchedulerBase::createConnectedGlContext(GLFWwindow *context) {
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);//4.6 is the least viable version we need
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
		shared_ptr<MeshReconstruction> map,  GpuStorage *gpu_storage,
		video::Source *source, GLFWwindow *context,
		LowDetailRenderer* low_detail_renderer,
		int skip_interval)
		: last_known_depth_pose_(Matrix4f::Identity()),
		  gpu_storage_(gpu_storage),
		  low_detail_renderer_(low_detail_renderer),
		  expand_interval_(30),
		  skip_interval_(skip_interval),
		  end_threads_(false),
		  paused_(false),
		  take_next_step_(false) {

	capture_thread_ = thread(&SchedulerLinear::captureWorker_, this, map, source, context);
	pthread_setname_np(capture_thread_.native_handle(),"capture");

	active_sets.resize(3); // 0 for update, 1 and 2 for rendering
}

SchedulerLinear::~SchedulerLinear() {
	end_threads_ = true;
	capture_thread_.join();
	cout << "DEBUG: scheduler destroyed" << endl;
}

void SchedulerLinear::captureWorker_(shared_ptr<MeshReconstruction> reconstruction,
                                     video::Source *source, GLFWwindow *context) {
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

//	reconstruction->initInGlLogicContext(); // nope

	InformationRenderer* information_renderer =
		new InformationRenderer();

	information_renderer->initInContext(gpu_storage_->garbage_collector_);

	information_renderer->initInContext(
			640,// TODO: don't hardcode this!!!
			480,
			gpu_storage_->garbage_collector_);

	GeometryUpdater* geometry_updater =
			new GeometryUpdater(gpu_storage_->garbage_collector_,640,480);
	TextureUpdater* texture_updater =
			new TextureUpdater();
	//texture_updater.mesh_reconstruction = map.get();
	geometry_updater->setup(reconstruction.get());



	Vector4f fc = source->intrinsicsDepth();

	//Camera tracking via cuda icp
	ICPOdometry *odometry = new ICPOdometry(640, 480, fc[2], fc[3], fc[0], fc[1]);

	IntermediateMap intermediate_map(640, 480, fc);

	reconstruction->setDepthIntrinsics(source->intrinsicsDepth());
	reconstruction->setRGBIntrinsics(source->intrinsicsRgb());

	//this is a replacement for a proper test if there already is something added to the map
	bool first_lap = true;

	Sophus::SE3d accu_pose;

	accu_pose.setRotationMatrix(Matrix3d::Identity());
	accu_pose.translation() = Vector3d(0, 0, 0);

	Matrix4f rel_depth_to_color = source->depthToRgb();

	shared_ptr<ActiveSet> active_set_last_expand;
	Matrix4f depth_pose_last_expand;

	while(source->isRunning() && !end_threads_) {
		if(paused_ && !take_next_step_) {
			continue;
		}
		take_next_step_ = false;
		if(!source->readFrame()) {
			break; // end this loop if there is no new image
			//This could be written more efficiently (but who cares about beautiful code?) Niko does.
		}

        if(((frame_count % (skip_interval_))) != 0 && !((frame_count % expand_interval_ == 0) || first_lap)){
            cout << "skip" << endl;
            frame_count++;
            continue;
        }
        cout << "no skip" << endl;

		cv::Mat depth = source->frame.depth; // 16 bit 1mm resolution
		cv::Mat rgb   = source->frame.rgb; // 8 bit 3 channels (usually)

		if(depth.type() != CV_16UC1) {
			assert(0);
		}

		cv::Mat depthu16;
		depth.convertTo(depthu16, CV_16UC1);//mm resolution needed needed vor ICPCUDA

		odometry->initICP((unsigned short*) depthu16.data, depth_cutoff);

		if(first_lap) {
			odometry->initICPModel((unsigned short*) depthu16.data, depth_cutoff);

		} else {
			//TODO: use information renderer to render this:
			active_sets_mutex.lock();
			shared_ptr<ActiveSet> render_set = active_sets[0];
			active_sets_mutex.unlock();
			//TODO: use information_renderer to render
			cv::Mat reprojected_depth =
					information_renderer->renderDepth(
							render_set,
							gpu_storage_,
							accu_pose.cast<float>().matrix(),
							reconstruction->params.depth_fxycxy);

			cv::imshow("rendered_depth",reprojected_depth);
			cv::waitKey(1);
			//information_renderer->renderDepth();//bla!



			//TODO: don't use rendering provided by reconstruction!!!!
			//cv::Mat reprojected_depth =
			//		reconstruction->generateDepthFromView(640, 480, information_renderer, accu_pose.cast<float>().matrix());

			odometry->initICPModel((unsigned short*) reprojected_depth.data, depth_cutoff);

			Sophus::SE3d rel_pose;
			odometry->getIncrementalTransformation(rel_pose, threads, blocks);
			accu_pose = accu_pose * rel_pose;
		}

		active_sets_mutex.lock();
		shared_ptr<ActiveSet> active_set_capturing = active_sets[0];
		active_sets_mutex.unlock();

		Matrix4f depth_pose = accu_pose.cast<float>().matrix();
		Matrix4f rgb_pose   = rel_depth_to_color * depth_pose;

		//If there is groundtruth trajectory loaded, we will use it!!!
		if(source->providesOdom()) {
			depth_pose = source->odom.pose;
			rgb_pose   = source->odom.pose; // TODO derive this from depthToRgb()
		}
		last_known_depth_pose_ = depth_pose;

		//upload the data to the gpu
		cv::Mat rgba;
		cv::cvtColor(rgb, rgba, cv::COLOR_BGR2RGBA);

		shared_ptr<gfx::GpuTex2D> rgb_texture = 
				make_shared<gfx::GpuTex2D>(gpu_storage_->garbage_collector_, GL_RGBA, GL_RGBA,
				                           GL_UNSIGNED_BYTE, rgba.cols, rgba.rows,
				                           true, rgba.data);
		rgb_texture->name = "[scheduler] rgb_texture";

		//do the same for depth!
		cv::Mat depthf;
		//the depth is in mm
		depth.convertTo(depthf, CV_32FC1, 1.0f / 1000.0f);
		//create the depth map and also the depth standardDeviation on the gpu. Further elements will use this
		shared_ptr<gfx::GpuTex2D> depth_tex = 
				make_shared<gfx::GpuTex2D>(gpu_storage_->garbage_collector_, GL_R32F, GL_RED, GL_FLOAT,
				                           depth.cols, depth.rows, true, 
				                           static_cast<void*>(depthf.data));
		depth_tex->name = "[scheduler] depth texture";

		//upload the data to the depth tex.
		shared_ptr<gfx::GpuTex2D> d_std_tex = 
				make_shared<gfx::GpuTex2D>(gpu_storage_->garbage_collector_, GL_RGBA32F, GL_RGBA,
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

		active_sets_mutex.lock();
		shared_ptr<ActiveSet> active_set = active_sets[0];
		active_sets_mutex.unlock();

		vector<shared_ptr<Meshlet>> visible_meshlets =
				reconstruction->getVisibleMeshlets(
						depth_pose,
						reconstruction->params.depth_fxycxy,
						reconstruction->params.depth_res,
						10.0f); // max distance the sensor can operate at

		//don't ask me what this is doing here!TODO: find out (or at least remove)
		//reconstruction->clearInvalidGeometry(active_set, depth, depth_pose);
		Matrix4f depth_proj = Camera::genProjMatrix(source->intrinsicsDepth());


		active_set =
				geometry_updater->update(
						gpu_storage_,
						information_renderer,
						visible_meshlets,
						active_set,
						this,
						d_std_tex,
						depth_pose,
						depth_proj);


		if(active_set!=nullptr){
			active_set->name = "geometry_updated_set";
		}






		active_sets_mutex.lock();
		active_sets[0] = active_set;
		active_sets_mutex.unlock();
        setActiveSetRendering(active_set);


		//TODO: test this by not applying texture in extend step
		active_set =
				texture_updater->colorTexUpdate(
						gpu_storage_,
						visible_meshlets,
						this,
						rgb_texture,
						reconstruction->params.rgb_fxycxy,
						rgb_pose);

		if(active_set!= nullptr){
			active_set->name = "color_updated_set";
		}

		active_sets_mutex.lock();
		active_sets[0] = active_set;
		active_sets_mutex.unlock();


		//every now and then we add new geometry:
		if((frame_count % expand_interval_ == 0) || first_lap) {

			//TODO: think if this should really take a Active Set or a list of visible meshes
			//expanding the existing geometry
			shared_ptr<ActiveSet> updated_geometry_set = geometry_updater->extend(
					this,
					reconstruction.get(), information_renderer, texture_updater, low_detail_renderer_,
					gpu_storage_,
					active_set, d_std_tex, d_std_mat, depth_pose,
					rgb_texture, rgb_pose);

			updated_geometry_set->name = "geometry update set";

			active_sets_mutex.lock();
			active_sets[0] = updated_geometry_set;
			active_sets_mutex.unlock();
            setActiveSetRendering(updated_geometry_set);

			//after the first step we wait (DEBUG).
			cv::waitKey(1);


			//active_set_last_expand = reconstruction->active_set_expand;
			depth_pose_last_expand = depth_pose;
		}

		frame_count++;

		//debug: printing the number of used vertices into a file

		first_lap = false;

		//cleanup VBO and VAOs that are deleted but only used within this thread
		//reconstruction->cleanupGlStoragesThisThread_();
		gpu_storage_->garbage_collector_->collect();

		cv::imshow("rgb", rgb);
		cv::waitKey(1);
		//tons of debug output to find this fucking memory leak!!!!
		int tex_count = gpu_storage_->tex_atlas_stds_->countTex() +
						gpu_storage_->tex_atlas_geom_lookup_->countTex() +
						gpu_storage_->tex_atlas_rgb_8_bit_->countTex();
		cout << "texCount overall: " << tex_count << " stds " <<
				gpu_storage_->tex_atlas_stds_->countTex() << " lookup " <<
				gpu_storage_->tex_atlas_geom_lookup_->countTex() << " rgb " <<
				gpu_storage_->tex_atlas_rgb_8_bit_->countTex() << endl;

		int patch_count = 	gpu_storage_->tex_atlas_stds_->countPatches() +
							gpu_storage_->tex_atlas_geom_lookup_->countPatches() +
							gpu_storage_->tex_atlas_rgb_8_bit_->countPatches();

		cout << "patchCount overall: " << patch_count << " stds " <<
				gpu_storage_->tex_atlas_stds_->countPatches() << " lookup " <<
				gpu_storage_->tex_atlas_geom_lookup_->countPatches() << " rgb " <<
				gpu_storage_->tex_atlas_rgb_8_bit_->countPatches() << endl;

		cout << "FBOs active " << gpu_storage_->fbo_storage_->getTotalFboCount() << endl;
	}

	delete information_renderer;
	delete geometry_updater;
	delete texture_updater;
	//delete everything for the fbo
	gpu_storage_->fbo_storage_->forceGarbageCollect();
	gpu_storage_->garbage_collector_->forceCollect();
	glFinish();



	glfwDestroyWindow(connected_context);
	delete odometry;

	cout << "DEBUG: scheduler finished thread" <<endl;
}


vector<shared_ptr<ActiveSet>> SchedulerLinear::getActiveSets(){
	active_sets_mutex.lock();
	vector<shared_ptr<ActiveSet>> sets = active_sets;
	active_sets_mutex.unlock();
	return sets;
}