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

out vec2 label_pos_out;
flat out uint64_t bindless_label_texture;

out float z;

out vec4 normal_out;

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	const GpuTriangle triangle = triangles[triangle_id];
	GpuPatchInfo main_patch_info = patches[triangle.patch_info_inds[0]];

	GpuPatchInfo patch_info = patches[triangle.patch_info_inds[point_id]];
	int vertex_id = triangle.pos_indices[point_id] +
	                patch_info.vertex_source_start_ind;
	vec4 point = vertices[vertex_id].p;

	GpuTextureInfo tex_info = main_patch_info.texture_infos[0];
	bindless_texture = tex_info.tex_pointer_gl;

	uint32_t tex_pos_ind = triangle.tex_indices[point_id] + 
	                       tex_info.tex_coord_start_ind;

	tex_pos_out = tex_coords[tex_pos_ind];
	//adapt the coordinate to the atlas
	tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x,
	                   tex_pos_out.y * tex_info.size.y);
	tex_pos_out = tex_pos_out + tex_info.pos;

	interp_position = view_matrix * point;  //new rendering
	interp_proj = proj_matrix * interp_position;
	z = 1.0 / interp_position.z;
	gl_Position = proj_matrix * interp_position;

	//setup the readout for the label texture;
	tex_info = main_patch_info.segmentation_texture;
	bindless_label_texture = tex_info.tex_pointer_gl;

	tex_pos_ind = triangle.tex_indices[point_id] + tex_info.tex_coord_start_ind;
	label_pos_out = tex_coords[tex_pos_ind];
	//adapt the coordinate to the atlas
	label_pos_out = vec2(tex_pos_out.x * tex_info.size.x,
	                     tex_pos_out.y * tex_info.size.y);
	label_pos_out = tex_pos_out + tex_info.pos;

	//calculate normal
	vec3 points[3];
	for(int i = 0; i < 3; i++) {
		GpuPatchInfo patch_info = patches[triangle.patch_info_inds[i]];
		int vertex_id = triangle.pos_indices[i] +
		                patch_info.vertex_source_start_ind;
		vec4 p = view_matrix * vertices[vertex_id].p;
		points[i] = p.xyz;
	}

	normal_out.xyz = normalize(cross(points[1] - points[0], points[2] - points[0]));
	normal_out.w = 1;
}
)"
