#include <gpu/gpu_storage.h>
#include <gpu/garbage_collector.h>
#include <gpu/thread_safe_FBO_VAO.h>
#include <gpu/tex_atlas.h>

#include <iostream>

#include <opencv2/core/cuda.hpp>

#include <gpu/active_set.h>

using namespace std;
using namespace Eigen;

/**
 * We need a new way of handling these slots,
 * Idea: there should be a queue of free slots from which we get free ones.
 * To ensure that they stay in gpu memory we create a new structure (BufferSlotConnector)
 * the last reference to BufferSlotConnector reinserts the utilized slots into the queue.
 * The patches and so on only store a weak_ptr the real shared_ptr should be stored within the ActiveSet Objects
 *
 */

void copy(cudaTextureObject_t texture, cv::cuda::GpuMat &to);

void GpuStorage::resetTimers() {
	time_spent_uploading_vertices = chrono::duration<double>::zero();
	upload_calls_vertices = 0;
	time_spent_uploading_tex_pos = chrono::duration<double>::zero();
	upload_calls_tex_pos = 0;
	time_spent_uploading_triangles = chrono::duration<double>::zero();
	upload_calls_triangles = 0;
	time_spent_uploading_patch_infos = chrono::duration<double>::zero();
	upload_calls_patch_infos = 0;
}

void GpuStorage::unloadMeshPatch_(Meshlet *patch) {
	cout << "the mechanism for unloading a mesh patch is not implemented yet" <<  endl;
	assert(0);
}

void GpuStorage::unloadDoubleStitch_(DoubleStitch *stitch) {
}

/**
 * @brief GpuGeomStorage::uploadTripleStitches
 * @param tripleStitches
 * Triple stitches are shit.....
 * We collect the triple stitches to put them collectively into triangle blocks. Whenever one of the triangles within one slot
 * becomes invalid, the slot should be refilled with the remaining valid triangles.
 * TODO: implement this! or maybe not? Probably not!!! (don't know anymore.... think about it
 */
void GpuStorage::uploadTripleStitches_(
		vector<TripleStitch*> triple_stitches) {
	cout << "[GpuStorage::uploadTripleStitches] for using this unimplemented method i sentence you to crash" << endl;
	assert(0);
}


GpuStorage::GpuStorage() {
	//TODO: delete these 2 lines from here,
	//this 1) is not necessary with only one gpu
	//and 2) it should happen way earlier in the code
	cudaSetDevice(0);

	//create the buffers that contain slots for our data (vertices, triangles, references to textures
	// and texture coordinates)
	vertex_buffer     = new GpuBuffer<GpuVertex>(max_nr_vertices);
	tex_pos_buffer    = new GpuBuffer<Vector2f>(max_nr_tex_coordinates);
	triangle_buffer   = new GpuBuffer<GpuTriangle>(max_nr_triangles);
	patch_info_buffer = new GpuBuffer<GpuPatchInfo>(max_nr_loaded_patch_infos);
	patch_info_index  = new GpuBuffer<GLint>(max_nr_loaded_patch_infos,
											 GL_ATOMIC_COUNTER_BUFFER);

	//setting up everything garbag ecollector related
	garbage_collector_ = new GarbageCollector();
	fbo_storage_ = new ThreadSafeFBOStorage();

	//setting up textures at this position
	//the garbage collector is missing here
	tex_atlas_geom_lookup_ = new TexAtlas(garbage_collector_, GL_RGBA32F,
										  GL_FLOAT, GL_RGBA, CV_32FC4,
										  1024, fbo_storage_);
	tex_atlas_stds_ = new TexAtlas(garbage_collector_, GL_RGBA16F,
											GL_FLOAT, GL_RGBA, CV_32FC4,
											1024, fbo_storage_);
	tex_atlas_rgb_8_bit_ = new TexAtlas(garbage_collector_, GL_RGBA,
												 GL_UNSIGNED_BYTE, GL_RGBA,
												 CV_8UC4, 1024, fbo_storage_);
}
GpuStorage::~GpuStorage() {
	if(vertex_buffer != nullptr) {
		delete vertex_buffer;
		delete tex_pos_buffer;
		delete triangle_buffer;
		delete patch_info_buffer;
		delete patch_info_index;
	}
	delete tex_atlas_geom_lookup_;
	delete tex_atlas_stds_;
	delete tex_atlas_rgb_8_bit_;

	delete garbage_collector_;
	delete fbo_storage_;
}

shared_ptr<ActiveSet> GpuStorage::makeActiveSet(
		vector<shared_ptr<Meshlet>> patches,
		MeshReconstruction *map,
		LowDetailRenderer* low_detail_renderer,
		TextureUpdater* texture_updater,
		InformationRenderer* information_renderer,
		bool initial, bool debug1) {
/*
	shared_ptr<ActiveSet> active_set =
			shared_ptr<ActiveSet>(
					new ActiveSet(this, patches, map, low_detail_renderer, texture_updater, information_renderer, initial, debug1));//instead of just patches
*/
	shared_ptr<ActiveSet> active_set;
	return active_set;
}
