#ifndef FILE_MESH_RECONSTRUCTION_H
#define FILE_MESH_RECONSTRUCTION_H

#include <map>
//synchronization
#include <condition_variable>
#include <atomic>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include "gpu/gpu_geom_storage.h"
#include "utils/octree.h"
#include "gpu/thread_safe_FBO_VAO.h"
#include "map_information_renderer.h"
#include "map_presentation_renderer.h"
#include "low_detail_map_renderer.h"
//TODO: migrate functionality to these new classes
#include "texturing.h"
#include "geometry_update.h"
#include "meshing.h"
#include "stitching.h"
#include "labelling.h"

using namespace std;
using namespace Eigen;

class PreSegmentation;
class GpuNormSeg;
class TexAtlas;

namespace gfx {
class GLSLProgram;
class GpuTex2D;
} // namespace gfx

class Worker;

class SchedulerLinear;
class SchedulerThreaded;;
struct Edge;

class MapInformationRenderer;
class MapPresentationRenderer;
class LowDetailRenderer;

class GarbageCollector;
class Exporter;

/**
 *Thoughts about spanning work over threads:
 * https://stackoverflow.com/questions/21010932/c11-parallel-for-implementation
 */
class MeshReconstruction {
	friend MapInformationRenderer;
	friend MapPresentationRenderer;
	friend LowDetailRenderer;
	friend Scheduler; //maybe we don't want this one as friend
	friend SchedulerLinear;
	friend SchedulerThreaded;
	friend ActiveSet;
	friend MeshTexture;
	friend Stitching;
	friend GeometryUpdate;
	friend Texturing;
	friend Labelling;
	friend Exporter;
	friend Meshing;

public:

	//TODO: use these to replace everything else
	struct Parameters {

		size_t max_sem_label_count = 8;
		cv::Size2i depth_res;
		cv::Size2i rgb_res;

		// These variables decides if and when new textures should be added or removed
		int max_nr_similar_textures = 3;
		float max_depth_factor_thresh_for_tex_adding = 0.6f;
		float max_depth_factor_thresh_for_tex_replacement = 0.3f;

		float max_angle_factor_thresh_for_tex_adding = 0.8f;
		float max_angle_factor_thresh_for_tex_replacement = 0.5f;

		float max_depth_step = 0.1f;
		float max_distance = 3.0f;//distance cutoff

		Vector4f rgb_fxycxy;
		Vector4f depth_fxycxy;
		Matrix4f cam_position;//maybe this should not be in here!!!

	};

	MeshReconstruction(GLFWwindow *context, GarbageCollector *garbage_collector,
	                   bool threaded, int depth_width = 640, 
	                   int depth_height = 480, int rgb_width = 640, 
	                   int rgb_height = 480);

	~MeshReconstruction();

	bool removePatch(shared_ptr<MeshPatch> patch);

	shared_ptr<MeshPatch> getPatchById(int id) {
		patches_mutex_.lock();
		shared_ptr<MeshPatch> patch = patches_[id];
		patches_mutex_.unlock();
		return patch;
	}
	vector<shared_ptr<MeshPatch>> GetAllPatches();

	void clearInvalidGeometry(shared_ptr<ActiveSet> set, cv::Mat depth, 
	                          Matrix4f depth_pose);

	float getMaxDistance() {
		return params.max_distance;
	}

	//TODO: these two
	shared_ptr<MeshPatch> genMeshPatch();
	shared_ptr<MeshTexture> genMeshTexture(MeshTexture::Type content_type);

	void setRGBIntrinsics(Vector4f fxycxy);
	void setDepthIntrinsics(Vector4f fxycxy);

	cv::Mat generateDepthFromView(int width, int height, Matrix4f pose);

	Matrix4f genDepthProjMat();

	/**
	 * @brief initInGlLogicContext
	 */
	void initInGlLogicContext();

	/**
	 * @brief initInGLRenderingContext
	 */
	void initInGLRenderingContext();

	bool hasGeometry();

	//pls describe these
	shared_ptr<ActiveSet> genActiveSetFromPose(Matrix4f depth_pose);
	vector<cv::Rect2f> genBoundsFromPatches(
			vector<shared_ptr<MeshPatch>> &patches, Matrix4f pose, 
			Matrix4f proj, shared_ptr<ActiveSet> active_set);

	//Free everything
	void erase();

	Parameters params;

	//TODO: remove, just for debugging purpose
	MapInformationRenderer information_renderer;
	MapPresentationRenderer render_presentation;
	LowDetailRenderer low_detail_renderer;

	GeometryUpdate geometry_update;
	Texturing texturing;
	Labelling labelling;

	mutex active_set_update_mutex;
	shared_ptr<ActiveSet> active_set_update;
	shared_ptr<ActiveSet> active_set_expand;

	atomic<bool> initializing_logic;

private:

	void cleanupGlStoragesThisThread_();

	int getFboCountDebug_() {
		return fbo_storage_.getTotalFboCount();
	}

	cv::Mat generateColorCodedTexture_(cv::Mat segmentation);

	void setActiveSetUpdate_(shared_ptr<ActiveSet> set);

	TriangleReference addTriangle_(
			VertexReference pr1, VertexReference pr2, VertexReference pr3, 
			vector<weak_ptr<GeometryBase>> &debug_new_stitches);
	TriangleReference addTriangle_(VertexReference pr1, VertexReference pr2, 
	                               VertexReference pr3);

	GarbageCollector *garbage_collector_;

	//shouldnt that be a shared pointer
	mutex patches_mutex_;
	map<int, shared_ptr<MeshPatch>> patches_;
	Octree<MeshPatch> octree_;//stores the objects in a spacial manner
	int current_max_patch_id_ = 0;//this basically is the number of patches currently in use

	//TODO: completely get rid of the concept of recycler
	ThreadSafeFBOStorage fbo_storage_;


	/**
	 * @brief preSeg
	 * This is supposed to be an interface for interchangeable pre segmentation
	 */
	shared_ptr<PreSegmentation> pre_seg_;
	shared_ptr<GpuNormSeg> gpu_pre_seg_;

	/****************THIS IS A VALID COMMENT THINK ABOUT IT******************/
	//
	//these are 4 channel 32bit floats right now
	//1: really this is an 32bit integer with an absolute triangle index
	//2,3,4: the barycentric coordinates on the triangle
	//TODO: reduce these to 4 channel 16 bit float
	//1,2: combined they are one 32Bit integer
	//3,4: 2 parts of the barycentric coordinate.
	//the third one is obtained by 1 - byrycentric1 -barycentric2
	shared_ptr<TexAtlas> tex_atlas_geom_lookup_;

	//Standard deviations of the surfaces
	shared_ptr<TexAtlas> tex_atlas_stds_;//[2];

	//at the moment we only store the SDR versions of the textures
	shared_ptr<TexAtlas> tex_atlas_rgb_8_bit_;


	//TODO check which is best for our task
	//pure 32 bit integers
	shared_ptr<TexAtlas> tex_atlas_seg_labels_;
	//int32 x 4 but interpreted as pairs of 16bit integers and
	//16 bit (float) weights
	shared_ptr<TexAtlas> tex_atlas_sem_seg_labels_weights_;

	GpuGeomStorage gpu_geom_storage_;

	//condition variable used to synchronize generation of the gpu buffer:
	//http://en.cppreference.com/w/cpp/thread/condition_variable/wait
	bool gl_logic_initialized_ = false;
	condition_variable condition_variable_;
	mutex condition_variable_mutex_;

	Worker *rendering_active_set_update_worker_ = nullptr;
	mutex active_set_rendering_mutex_;
	shared_ptr<ActiveSet> active_set_rendering_;

};

#endif // FILE_MESH_RECONSTRUCTION_H
