R"(
//The data is delivered in buffers
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
out vec2 tex_pos_out;
out vec4 interp_position;
out vec4 interp_proj;
flat out uint64_t bindless_texture;
flat out int is_stitch;
flat out int patch_id;
flat out int tex_coord_slot_out;//debug
out float z;

vec2 res = vec2(1280, 800);
out float distances[3];

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	const GpuTriangle triangle = triangles[triangle_id];
	GpuPatchInfo main_patch_info = patches[triangle.patch_info_inds[0]];
	patch_id = main_patch_info.patch_id;

	GpuPatchInfo patch_info = patches[triangle.patch_info_inds[point_id]];
	int vertex_id = triangle.pos_indices[point_id] +
	                patch_info.vertex_source_start_ind;
	vec4 point = vertices[vertex_id].p;
	GpuTextureInfo tex_info = main_patch_info.std_texture;
	tex_coord_slot_out = int(tex_info.tex_coord_start_ind);//debug seems to be OK tough
	bindless_texture = tex_info.gl_tex_pointer;

	is_stitch = 1;//unfortunately as it is right now we can't tell if a triangle is stitching
	//------------------------------------------

	uint32_t tex_pos_ind = triangle.tex_indices[point_id] + 
	                       tex_info.tex_coord_start_ind;

	tex_pos_out = tex_coords[tex_pos_ind];
	tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x, 
	                   tex_pos_out.y * tex_info.size.y);
	tex_pos_out = tex_pos_out + tex_info.pos;

	interp_position = view_matrix * point;  //new rendering
	interp_proj = proj_matrix * interp_position;
	z = 1.0 / interp_position.z;
	gl_Position = proj_matrix * interp_position;
}
)"
