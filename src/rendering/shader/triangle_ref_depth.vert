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

layout(location = 0) uniform mat4 view_matrix;//one matrix is taking up 4 locations (i don't really think this is true anymore)
layout(location = 1) uniform mat4 proj_matrix;
layout(location = 2) uniform int patch_info_start_ind;
//some funny varyings altough some are flat
flat out int patch_id;
flat out int triangle_index;
out vec4 point_world;

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	GpuPatchInfo patch_info = patches[patch_info_start_ind + gl_DrawID];
	const GpuTriangle triangle = triangles[triangle_id + patch_info.triangle_start_ind];
	patch_id = patch_info.patch_id;
	triangle_index = triangle_id;

	int vertex_id = triangle.indices[point_id] +
	                patch_info.vertex_start_ind;
	vec4 point = vertices[vertex_id].p;
	point_world = point;

	vec4 interp_position = view_matrix * point;  //the position of the vertex in space (gets interpolated for the fragments)
	gl_Position = proj_matrix * interp_position;
	gl_Position.y *= -1;

	//Debug
	//gl_Position = point;
	//gl_Position.xyz *= 0.2;
}
)"
