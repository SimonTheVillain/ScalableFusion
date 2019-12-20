R"(
//the data comes in via array buffers
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

//layout(location = 0) uniform int max_nr_tex_points; //this seems to be the only thing that is actually needed
//layout(location = 1) uniform int patch_info_start_ind;
layout(location = 0) uniform int start_tris;
layout(location = 1) uniform int start_tex_coords;
//todo: The output for the whole shader is going to be a texture with triangle indices and
//a parametrized position on the textures
flat out int tri_ind;
flat out int vert_ind1;
flat out int vert_ind2;
flat out int vert_ind3;
out vec3 barycentric_weights;//this is not (barycentric interpolation?)

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	tri_ind = triangle_id;
	const GpuTriangle triangle = triangles[triangle_id + start_tris];

	vert_ind1 = triangle.indices[0];
	vert_ind2 = triangle.indices[1];
	vert_ind3 = triangle.indices[2];


	int ind = triangle.indices[point_id];


	//the new stuff:
	gl_Position = vec4(tex_coords[ind + start_tex_coords] * 2.0 - vec2(1.0, 1.0), 0, 1);

	//TODO: test if this really does barycentric interpolation in opengl
	barycentric_weights = vec3(0, 0, 0);
	barycentric_weights[point_id] = 1;
}
)"
