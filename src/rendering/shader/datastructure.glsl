R"(
#version 460 core
#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_NV_gpu_shader5 : enable

//this is a quasiheader that defines the datastructures that are used by the opengl
//headers

//TODO: change that to something reasonable: 3 maybe?
const int GPU_MAX_TEX_PER_PATCH = 10;

struct GpuVertex {
	vec4 p;
	vec3 n;
	int16_t padding;
	//int16_t tex_index; //the texture index within the texture associated with its patch
	int16_t valid;
};

struct GpuTriangle {
	int16_t indices[3];
};



//POSSIBILITY: Adding cuda surfaces and textures could increase utility of this
//structure
struct GpuTextureInfo {
	uint64_t ref_tex_ptr_debug_gl;
	vec2     ref_tex_pos_debug;
	uint64_t tex_pointer_gl;
	uint32_t tex_coord_start_ind;//could also be 16 bit int
	uint32_t padding;

	/**
	 * Since the texture itself is shared with multiple other patches
	 * we also need to store where the texture is on this texture atlas
	 */
	//don't know yet what will be needed afterall
	vec2 pos;
	vec2 size;
	vec2 _size;
};

//https://www.ibm.com/developerworks/aix/tutorials/au-memorymanager/
/**
 * Not a big structure that does not contain much for the triangles themselves,
 * only offsets and other stuff that might be useful not to do on the cpu.
 * But maybe it is not useful to calculate these offsets for each of the render passes.
 * maybe we first create an easily renderable structure via a cuda shader. (maybe by
 * storing vertex indices in a ringbuffer)
 */
struct GpuPatchInfo {
	int32_t patch_id; //only for information rendering i presume
	int32_t triangle_start_ind;
	int32_t vertex_start_ind;
	GpuTextureInfo std_texture;
	GpuTextureInfo texture_infos[GPU_MAX_TEX_PER_PATCH];
	int32_t tex_layers;

	//TODO!!

	//int32_t debug1;
	///maybe we should add some flags for cases that one of the cuda kernels updated some of the
	/// geometry. This could easily be detected and give the chance to download updated geometry to the
	/// cpu.
};
)"
