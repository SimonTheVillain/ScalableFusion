#ifndef FILE_MESH_RECONSTRUCTION_H
#define FILE_MESH_RECONSTRUCTION_H

#include <unordered_map>
//synchronization
#include <condition_variable>
#include <atomic>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>

#include <gpu/gpu_storage.h>
#include <utils/octree.h>
#include <gpu/thread_safe_FBO_VAO.h>
#include <rendering/information_renderer.h>
#include <rendering/presentation_renderer.h>
#include <rendering/low_detail_renderer.h>
//TODO: migrate functionality to these new classes
#include <texture_updater.h>
#include <geometry_updater.h>
#include <mesher.h>
#include <mesh_stitcher.h>
#include <labelling.h>

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

class InformationRenderer;
class PresentationRenderer;
class LowDetailRenderer;

class GarbageCollector;
class MapExporter;


/**
 *Thoughts about spanning work over threads:
 * https://stackoverflow.com/questions/21010932/c11-parallel-for-implementation
 */
class MeshReconstruction {
	friend InformationRenderer;
	friend PresentationRenderer;
	friend LowDetailRenderer;
	friend Scheduler; //maybe we don't want this one as friend
	friend SchedulerLinear;
	friend SchedulerThreaded;
	friend ActiveSet;
	friend MeshTexture;
	friend MeshStitcher;
	friend GeometryUpdater;
	friend TextureUpdater;
	friend Labelling;
	friend MapExporter;
	friend Mesher;

public:

	//TODO: this should not be in the reconstruction itself
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
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

	MeshReconstruction(int depth_width = 640,
	                   int depth_height = 480, int rgb_width = 640, 
	                   int rgb_height = 480);

	~MeshReconstruction();

	bool removePatch(shared_ptr<Meshlet> patch);

	shared_ptr<Meshlet> getPatchById(int id) {
		meshlet_mutex_.lock();
		shared_ptr<Meshlet> patch = meshlets_[id];
		meshlet_mutex_.unlock();
		return patch;
	}
	vector<shared_ptr<Meshlet>> GetAllPatches();

	void clearInvalidGeometry(shared_ptr<ActiveSet> set, cv::Mat depth, 
	                          Matrix4f depth_pose);

	float getMaxDistance() {
		return params.max_distance;
	}

	//TODO: these two
	shared_ptr<Meshlet> genMeshlet();
	shared_ptr<MeshTexture> genMeshTexture(MeshTexture::Type content_type);

	void setRGBIntrinsics(Vector4f fxycxy);
	void setDepthIntrinsics(Vector4f fxycxy);

	cv::Mat generateDepthFromView(int width, int height,
								  InformationRenderer* renderer,
								  Matrix4f pose);

	Matrix4f genDepthProjMat();

	/**
	 * @brief initInGlLogicContext
	 */
	//void initInGlLogicContext();

	/**
	 * @brief initInGLRenderingContext
	 */
	//void initInGLRenderingContext();

	bool hasGeometry();

	void checkNeighbourhoodConsistency();
	void checkTriangleVerticesConsistency();
	void checkLeftoverEdges();

	//pls describe these
	/*
	shared_ptr<ActiveSet> genActiveSetFromPose(Matrix4f depth_pose,
											   LowDetailRenderer* low_detail_renderer,
											   TextureUpdater* texture_updater,
											   InformationRenderer* information_renderer);
	 */
	vector<shared_ptr<Meshlet>> getVisibleMeshlets(
			Matrix4f pose,
			Vector4f intrinsics,
			cv::Size2i res,
			float max_dist);
	 vector<cv::Rect2f> genBoundsFromPatches(
			vector<shared_ptr<Meshlet>> &patches, Matrix4f pose,
			Matrix4f proj, shared_ptr<ActiveSet> active_set);

	//Free everything (forget it, freeing cpu resources should work by deleting gpu storgae
	// also the forced download of geometry should be done by that
	//void erase();

	Parameters params;
	shared_ptr<Meshlet> getMeshlet(int id);
	//TODO: remove, just for debugging purpose
	//InformationRenderer information_renderer;
	//PresentationRenderer render_presentation;
	//LowDetailRenderer low_detail_renderer;

	//GeometryUpdater geometry_update;
	//TextureUpdater texturing;
	//Labelling labelling;

	/*
	mutex active_set_update_mutex;
	shared_ptr<ActiveSet> active_set_update;
	shared_ptr<ActiveSet> active_set_expand;
	*/
	//atomic<bool> initializing_logic;

private:

	//void cleanupGlStoragesThisThread_();

	//TODO: this should not be in the reconstruction itself
	cv::Mat generateColorCodedTexture_(cv::Mat segmentation);

	//TODO: this should not be in the reconstruction itself
	//void setActiveSetUpdate_(shared_ptr<ActiveSet> set);

	Triangle* addTriangle_(
			Vertex* v1, Vertex* v2, Vertex* v3,
			vector<weak_ptr<GeometryBase>> &debug_new_stitches);
	Triangle* addTriangle_(Vertex* pr1, Vertex* pr2,
						   Vertex* pr3);

	//GarbageCollector *garbage_collector_;

	//shouldnt that be a shared pointer
	mutex meshlet_mutex_;
	unordered_map<int, shared_ptr<Meshlet>> meshlets_;
	octree::Octree octree_;//stores the objects in a spacial manner
	int current_max_patch_id_ = 0;//this basically is the number of patches currently in use

	//TODO: completely get rid of the concept of recycler
	//ThreadSafeFBOStorage fbo_storage_;


	/**
	 * @brief preSeg
	 * This is supposed to be an interface for interchangeable pre segmentation
	 */
	//shared_ptr<GpuNormSeg> gpu_pre_seg_;//TODO: move this to the geometry updater class

	/****************THIS IS A VALID COMMENT THINK ABOUT IT******************/
	//
	//these are 4 channel 32bit floats right now
	//1: really this is an 32bit integer with an absolute triangle index
	//2,3,4: the barycentric coordinates on the triangle
	//TODO: reduce these to 4 channel 16 bit float
	//1,2: combined they are one 32Bit integer
	//3,4: 2 parts of the barycentric coordinate.
	//the third one is obtained by 1 - byrycentric1 -barycentric2
	/*
	shared_ptr<TexAtlas> tex_atlas_geom_lookup_;

	//Standard deviations of the surfaces
	shared_ptr<TexAtlas> tex_atlas_stds_;//[2];

	//at the moment we only store the SDR versions of the textures
	shared_ptr<TexAtlas> tex_atlas_rgb_8_bit_;
	*/


	//GpuStorage gpu_geom_storage_;

	//condition variable used to synchronize generation of the gpu buffer:
	//http://en.cppreference.com/w/cpp/thread/condition_variable/wait
	/*
	bool gl_logic_initialized_ = false;
	condition_variable condition_variable_;
	mutex condition_variable_mutex_;

	Worker *rendering_active_set_update_worker_ = nullptr;
	mutex active_set_rendering_mutex_;
	shared_ptr<ActiveSet> active_set_rendering_;
	*/
};

#endif // FILE_MESH_RECONSTRUCTION_H
