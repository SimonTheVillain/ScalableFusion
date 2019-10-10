R"(
//the new source of data!!!
layout(std430, binding = 0) buffer VertexBuffer {
	GpuVertex vertices[];
};
layout(std430, binding = 1) buffer TexCoordBuffer {
	vec2 tex_coords[];
};
layout(std430, binding = 2) buffer TriangleBuffer {
	GpuTriangle triangles[];
};
layout(std430, binding = 3) buffer PatchBuffer {
	GpuPatchInfo patches[];
};

layout(location = 0) uniform mat4 view_matrix;//one matrix is taking up 4 locations
layout(location = 1) uniform mat4 proj_matrix;

//some funny varyings altough some are flat
flat out int is_stitch;
flat out int patch_id;
flat out int triangle_index;
out vec4 point_world;

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	const GpuTriangle triangle = triangles[triangle_id];
	GpuPatchInfo main_patch_info = patches[triangle.patch_info_inds[0]];
	patch_id = main_patch_info.patchId;
	triangle_index = triangle_id - main_patch_info.triangle_start_ind;

	GpuPatchInfo patch_info = patches[triangle.patch_info_inds[point_id]];
	int vertex_id = triangle.pos_indices[point_id] +
	                patch_info.vertex_source_start_ind;
	vec4 point = vertices[vertex_id].p;
	point_world = point;
	is_stitch = 10;

	vec4 interp_position = view_matrix*point;  //the position of the vertex in space (gets interpolated for the fragments)
	gl_Position = proj_matrix * interp_position;
	gl_Position.y *= -1;
}
)"
