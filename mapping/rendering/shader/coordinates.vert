R"(
//the data comes in via array buffers
layout(std430, binding = 0) buffer VertexBuffer {
	GpuVertex vertices[];
};

layout(std430, binding = 1) buffer TexCoordBuffer {
	vec2 texCoords[];
};

layout(std430, binding = 2) buffer TriangleBuffer {
	GpuTriangle triangles[];
};

layout(std430, binding = 3) buffer PatchBuffer {
GpuPatchInfo patches[];
};

layout(location = 0) uniform int max_nr_tex_points; //this seems to be the only thing that is actually needed


//todo: The output for the whole shader is going to be a texture with triangle indices and
//a parametrized position on the textures
flat out int triangle_index;//thats an easy one
out vec3 barycentric_weights;//this is not (barycentric interpolation?)

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int pointId     = id % 3;
	int triangle_id = id / 3;
	const gpu_triangle triangle = triangles[triangle_id];
	int patch_slot              = triangle.patch_info_inds[0];
	GpuPatchInfo patch_info     = patches[patchSlot];

	vec4 point = vertices[triangle.pos_indices[pointId]].p;

	uint32_t tex_pos_ind = triangle.tex_indices[pointId] +
	                       patch_info.std_texture.tex_coord_start_ind;

	//the new stuff:
	gl_Position = vec4(tex_coords[tex_pos_ind] * 2.0 - vec2(1.0, 1.0), 0, 1);
	triangle_index = triangle_id;

	//TODO: test if this really does barycentric interpolation in opengl
	barycentric_weights = vec3(0,0,0);
	barycentric_weights[pointId] = 1;
}
)"
