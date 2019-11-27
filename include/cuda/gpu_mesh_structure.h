#ifndef FILE_GPU_MESH_STRUCTURE_H
#define FILE_GPU_MESH_STRUCTURE_H

#include <stdint.h>

#include <Eigen/Eigen>

#define GPU_MAX_TEX_PER_PATCH 10

//TODO: remove this using namespace... or is there a reason to use this in a HEADER?
using namespace Eigen;

// TODO: 
//   maybe p could be a Vector3f,
//   n could be 3 bytes and
//   the tex index in main patch..... if this would not be used it could half
//   the space needed (forget it then!)
struct GpuVertex {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4f p;
	Vector3f n;

	// The texture index in the tex coordinates of the main patch
	int16_t tex_ind_in_main_patch;//could also be 16 bit int

	// If any of a triangle vertex is invalid we should not render it
	int16_t valid = 1;
};

//triangle with references to everything. since the first referenced patch(patch slot) is the one we get
//our textures from we do not need extra structures for stitch triangles.
struct GpuTriangle {
	int16_t indices[3];//these are absolute indices for the vertex buffer.

};


///TODO: maybe we add slots for CUDA surfaces and stuff like that.
struct GpuTextureInfo {
	uint64_t gl_ref_tex_ptr_DEBUG; // only used for rendering a debug view
	Vector2f ref_tex_pos_DEBUG; // same


	uint64_t gl_tex_pointer;
	uint32_t tex_coord_start_ind;//could also be 16 bit int
	uint32_t padding;

	/**
	 * Since the texture itself is shared with multiple other patches
	 * we also need to store where the texture is on this
	 */
	//don't know yet what will be needed afterall
	Vector2f pos;
	Vector2f size;
	Vector2f _size;

};
//there is some padding/ alignment issues!

//https://www.ibm.com/developerworks/aix/tutorials/au-memorymanager/
/**
 * Not a big structure that does not contain much for the triangles themselves,
 * only offsets and other stuff that might be useful not to do on the cpu.
 * But maybe it is not useful to calculate these offsets for each of the render passes.
 * maybe we first create an easily renderable structure via a cuda kernel. (maybe by
 * storing vertex indices in a ringbuffer)
 */
struct GpuPatchInfo {
	int32_t patch_id;//only for information rendering i presume
	int32_t triangle_start_ind;
	int32_t vertex_start_ind;
	GpuTextureInfo std_texture;
	GpuTextureInfo texture_infos[GPU_MAX_TEX_PER_PATCH];
	int32_t tex_layers;

	//int32_t debug1 = 0;
};

struct GpuCoarseVertex {
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Vector4f p;
	Vector4f n;
	Vector4f c;
};

#endif // FILE_GPU_MESH_STRUCTURE_H
