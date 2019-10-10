R"(
//the new source of data!!!
layout(std430, binding = 0) buffer VertexBuffer{
	GpuVertex vertices[];
};

layout(std430, binding = 1) buffer TexCoordBuffer{
	vec2 tex_coords[];
};
layout(std430, binding = 2) buffer TriangleBuffer{
	GpuTriangle triangles[];
};
layout(std430, binding = 3) buffer PatchBuffer{
	GpuPatchInfo patches[];
};

layout (location = 0) uniform mat4 view_matrix;//one matrix is taking up 4 locations
layout (location = 1) uniform mat4 proj_matrix;
layout (location = 4) uniform int render_wireframe;
layout (location = 5) uniform int color_mode;
layout (location = 6) uniform int lighting_mode;

//some funny varyings altough some are flat
out vec2 tex_pos_out;
out vec4 interp_position;
out vec4 interp_proj;
out vec4 normal;
flat out uint64_t bindless_texture;
flat out int is_stitch;
flat out int patch_id;
out float z;

flat out int32_t debug1;
flat out int32_t debug2;

vec2 res = vec2(1280, 800);//this should be a uniform set outside of the shader
out float distances[3];
void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	const GpuTriangle triangle = triangles[triangle_id];

	GpuPatchInfo main_patch_info = patches[triangle.patch_info_inds[0]];
	patch_id = main_patch_info.patch_id;
	debug1 = main_patch_info.debug1;

	GpuPatchInfo patch_info = patches[triangle.patch_info_inds[point_id]];
	int vertex_id = triangle.pos_indices[point_id] +
	                patch_info.vertex_source_start_ind;
	vec4 point = vertices[vertex_id].p;
	normal.xyz = vertices[vertex_id].n;

	GpuTextureInfo tex_info = main_patch_info.texture_infos[0];
	if(color_mode == 2) {
		tex_info = main_patch_info.std_texture;
	}

	if(main_patch_info.tex_layers == 0 && color_mode == 0) {
		bindless_texture = 0;
	} else {
		bindless_texture = tex_info.gl_tex_pointer;//take the texutre from first slot
	}

	is_stitch = 1;//unfortunately as it is right now we can't tell if a triangle is stitching
	//------------------------------------------

	uint32_t tex_pos_ind = uint32_t(triangle.tex_indices[point_id]) +
	                       tex_info.tex_coord_start_ind;
	tex_pos_out = tex_coords[tex_pos_ind];
	//adapt the coordinate to the tex atlas
	tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x,
	                   tex_pos_out.y * tex_info.size.y);
	tex_pos_out = tex_pos_out + tex_info.pos;
	if(color_mode == 4) {
		//in this case we put out the (COME ON WHY CAN'T YOU FINISH COMMENTS)
		tex_info = main_patch_info.std_texture;
		bindless_texture = tex_info.gl_ref_tex_ptr_debug;
		uint32_t tex_pos_ind = uint32_t(triangle.tex_indices[point_id]) +
		                       tex_info.tex_coord_start_ind;
		tex_pos_out = tex_coords[tex_pos_ind];
		//adapt the coordinate to the tex atlas
		tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x,
		                   tex_pos_out.y * tex_info.size.y);
		tex_pos_out = tex_pos_out + tex_info.ref_tex_pos_debug;
	}
	if(color_mode == 5) {
		//mode 5 is the output of the first layer of label textures
		tex_info = main_patch_info.segmentation_texture;
		bindless_texture = tex_info.gl_tex_pointer;
		uint32_t tex_pos_ind = uint32_t(triangle.tex_indices[point_id]) +
		                       tex_info.tex_coord_start_ind;
		tex_pos_out = tex_coords[tex_pos_ind];
		//adapt the coordinate to the tex atlas
		tex_pos_out = vec2(tex_pos_out.x * tex_info.size.x,
		                   tex_pos_out.y * tex_info.size.y);
		tex_pos_out = tex_pos_out + tex_info.ref_tex_pos_debug;

		bindless_texture = tex_info.gl_ref_tex_ptr_debug;
	}

	interp_position = view_matrix * point;  //the position of the vertex in space (gets interpolated for the fragments)
	interp_proj = proj_matrix * interp_position;
	z = 1.0 / interp_position.z;
	gl_Position = proj_matrix * interp_position;

	debug2 = 0;

	//for rendering the wireframe overlay: get points in screen coordinates
	//OpenGL 4 Shading Language cookbook page 231!!!!
	vec2 sc[3];//screen coords
	vec3 points[3];
	for(int i = 0; i < 3; i++) {
		GpuPatchInfo patch_info = patches[triangle.patch_info_inds[i]];
		int vertex_id = triangle.pos_indices[i] +
		                patch_info.vertex_source_start_ind;
		vec4 p = vertices[vertex_id].p;
		points[i] = p.xyz;
		p = proj_matrix * view_matrix * p;
		sc[i] = vec2(p.xy / p.w);
		sc[i].x = sc[i].x * res.x;
		sc[i].y = sc[i].y * res.y;

		//checck if the vertex to this triangle is valid
		if(int(vertices[vertex_id].valid) == 0) {
		}
	}

	//calculate heights
	float a = length(sc[1] - sc[2]);
	float b = length(sc[2] - sc[0]);
	float c = length(sc[1] - sc[0]);
	float alpha = acos((b * b + c * c - a * a) / (2.0 * b * c));
	float beta  = acos((a * a + c * c - b * b) / (2.0 * a * c));
	distances[0] = abs(c * sin(beta));//ha
	distances[1] = abs(c * sin(alpha));//hb
	distances[2] = abs(b * sin(alpha));//hc

	//WHAAAAAT?
	//setting two of them to zero
	for(int i = 0; i < 3; i++) {
		//if we are on point 0 the distance to line a would be non zero(=ha)
		//but the distances to the lines b and c would be zero.
		if(i != point_id) {
			distances[i] = 0;
		}
	}

	if(lighting_mode == 1 || true) { //always create the normal
		//for flat shading we need the normal of the triangle
		normal.xyz = normalize(cross(points[1] - points[0], points[2] - points[0]));
		normal.w = 1;
	}
}
)"
