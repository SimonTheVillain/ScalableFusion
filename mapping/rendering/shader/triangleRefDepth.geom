R"(
#version 450 core

#extension GL_ARB_gpu_shader_int64 : enable
#extension GL_NV_gpu_shader5 : enable

//this is a quasiheader that defines the datastructures that are used by the opengl
//headers

const int GPU_MAX_TEX_PER_PATCH = 100;

struct GpuVertex {
 vec4 p;
 vec3 n;
 int tex_index; //the texture index within the texture associated with its patch
};

struct GpuTriangle {
	int16_t patch_info_slots[3];
	int16_t pos_indices[3];
	int16_t tex_indices[3];
};

struct GpuTextureInfo {
	uint64_t gl_tex_pointer;
	uint32_t tex_coord_slot;
	uint32_t placeholder;

	vec2 pos;
	vec2 size;
	vec2 _size;

};

struct GpuPatchInfo {
	int32_t patch_id;
	int32_t triangle_slot;
	int32_t vertex_source_slot;
	int32_t vertex_destination_slot;
	GpuTextureInfo std_texture;
	GpuTextureInfo texture_infos[GPU_MAX_TEX_PER_PATCH];
	int32_t tex_layers;
	int32_t placeholder;
	///maybe we should add some flags for cases that one of the cuda kernels updated some of the
	/// geometry. This could easily be detected and give the chance to download updated geometry to the
	/// cpu.
};

struct Uniforms {//this will take up 2 locations
	int32_t nr_tex_points_per_slot;
	int32_t nr_vertices_per_slot;
};
)"
