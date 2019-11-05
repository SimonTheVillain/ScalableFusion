#include "gpu_geom_storage.h"

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

void GpuGeomStorage::resetTimers() {
	time_spent_uploading_vertices = chrono::duration<double>::zero();
	upload_calls_vertices = 0;
	time_spent_uploading_tex_pos = chrono::duration<double>::zero();
	upload_calls_tex_pos = 0;
	time_spent_uploading_triangles = chrono::duration<double>::zero();
	upload_calls_triangles = 0;
	time_spent_uploading_patch_infos = chrono::duration<double>::zero();
	upload_calls_patch_infos = 0;
}

void GpuGeomStorage::unloadMeshPatch_(MeshPatch *patch) {
	cout << "the mechanism for unloading a mesh patch is not implemented yet" <<  endl;
	assert(0);
}

void GpuGeomStorage::unloadDoubleStitch_(DoubleStitch *stitch) {
}

/**
 * @brief GpuGeomStorage::uploadTripleStitches
 * @param tripleStitches
 * Triple stitches are shit.....
 * We collect the triple stitches to put them collectively into triangle blocks. Whenever one of the triangles within one slot
 * becomes invalid, the slot should be refilled with the remaining valid triangles.
 * TODO: implement this! or maybe not? Probably not!!! (don't know anymore.... think about it
 */
void GpuGeomStorage::uploadTripleStitches_(
		vector<TripleStitch*> triple_stitches) {
	cout << "[GpuGeomStorage::uploadTripleStitches] for using this unimplemented method i sentence you to crash" << endl;
	assert(0);
}

void GpuGeomStorage::initialize() {
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

}

GpuGeomStorage::~GpuGeomStorage() {
	if(vertex_buffer != nullptr) {
		delete vertex_buffer;
		delete tex_pos_buffer;
		delete triangle_buffer;
		delete patch_info_buffer;
		delete patch_info_index;
	}
}

shared_ptr<ActiveSet> GpuGeomStorage::makeActiveSet(
		vector<shared_ptr<MeshPatch>> patches,
		MeshReconstruction *map,
		LowDetailRenderer* low_detail_renderer,
		TextureUpdater* texture_updater,
		InformationRenderer* information_renderer,
		bool initial, bool debug1) {

	shared_ptr<ActiveSet> active_set =
			shared_ptr<ActiveSet>(
					new ActiveSet(this, patches, map, low_detail_renderer, texture_updater, information_renderer, initial, debug1));//instead of just patches

	return active_set;
}
