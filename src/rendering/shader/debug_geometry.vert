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
layout(location = 3) uniform int  overwrite_start_ind; //TODO: remove this
//TODO: new inputs
layout(location = 4) uniform int patch_info_start_ind;

vec2 res = vec2(1280, 800);//this should be a uniform set outside of the shader
out float distances[3];

void main(void) {
	//-------------new rendering----------------
	int id          = gl_VertexID;
	int point_id    = id % 3;
	int triangle_id = id / 3;
	const GpuTriangle triangle = triangles[triangle_id];

	GpuPatchInfo patch_info = patches[patch_info_start_ind + gl_DrawID];
	int start_ind = patch_info.vertex_start_ind;


	if(overwrite_start_ind >= 0) {
		start_ind = overwrite_start_ind;
	}

	int vertex_id = triangle.indices[point_id] + start_ind;
	vec4 point = vertices[vertex_id].p;

	gl_Position = proj_matrix * view_matrix * point;

	vec2 sc[3];//screen coords
	vec3 points[3];
	for(int i = 0; i < 3; i++) {
		int vertex_id = triangle.indices[i] + start_ind;
		vec4 p = vertices[vertex_id].p;
		points[i] = p.xyz;
		p = proj_matrix * view_matrix * p;
		sc[i] = vec2(p.xy / p.w);
		sc[i].x = sc[i].x * res.x;
		sc[i].y = sc[i].y * res.y;

		if(int(vertices[vertex_id].valid) == 0) {
			//just for debug, draw triangles in certain color
			//sort of discard the whole triangle
			gl_Position.w = 0;
			return;
		}
	}

	float a = length(sc[1] - sc[2]);
	float b = length(sc[2] - sc[0]);
	float c = length(sc[1] - sc[0]);
	float alpha = acos((b * b + c * c - a * a) / (2.0 * b * c));
	float beta  = acos((a * a + c * c - b * b) / (2.0 * a * c));
	distances[0] = abs(c * sin(beta));//ha
	distances[1] = abs(c * sin(alpha));//hb
	distances[2] = abs(b * sin(alpha));//hc

	for(int i = 0; i < 3; i++) {
		//if we are on point 0 the distance to line a would be non zero(=ha)
		//but the distances to the lines b and c would be zero.
		if(i != point_id) {
			distances[i] = 0;
		}
	}
}
)"
