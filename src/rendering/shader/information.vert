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
layout(location = 2) uniform int patch_info_start_ind;

//some funny varyings altough some are flat
out vec2 tex_pos_out;
//out vec4 interp_position;
//out vec4 interp_proj;
out vec4 pos_cam;
flat out uint64_t bindless_texture;
flat out int patch_id;
flat out int tex_coord_slot_out;//debug
out float z;

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	GpuPatchInfo patch_info = patches[patch_info_start_ind + gl_DrawID];
	const GpuTriangle triangle = triangles[triangle_id+patch_info.triangle_start_ind];
	patch_id = patch_info.patch_id;

	int vertex_id = triangle.indices[point_id] +
	                patch_info.vertex_start_ind;
	vec4 point = vertices[vertex_id].p;
	GpuTextureInfo tex_info = patch_info.std_texture;
	tex_coord_slot_out = int(tex_info.tex_coord_start_ind);//debug seems to be OK tough
	bindless_texture = tex_info.tex_pointer_gl;

	//------------------------------------------

	uint32_t tex_pos_ind = triangle.indices[point_id] +
	                       tex_info.tex_coord_start_ind;

	tex_pos_out = tex_coords[tex_pos_ind];
	tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x, 
	                   tex_pos_out.y * tex_info.size.y);
	tex_pos_out = tex_pos_out + tex_info.pos;

	pos_cam = view_matrix * point; //vertex positions in camera space!
	//interp_proj = pos_cam;
	z = 1.0 / pos_cam.z;
	gl_Position = proj_matrix * pos_cam;
	//gl_Position = pos_cam;
	//gl_Position.xyz *= 0.7;
	//gl_Position.z *=0.2;
}
)"
