R"(
#version 450 core

layout(triangles) in;

layout(triangle_strip, max_vertices = 3) out;

in vec4 colors[];
in vec4 normals[];
in int  valids[];

out vec4 color_in;
out vec4 normal_in;

void main() {
	if(valids[0] == 0 && valids[1] == 0 && valids[2] == 0) {
		//don't do anything if the vertices are not all valid
		return; //debug
	}
	for(int i = 0; i < 3; i++) {
		color_in    = colors[i];
		normal_in   = normals[i];
		gl_Position = gl_in[i].gl_Position;

		EmitVertex();
	}

	EndPrimitive();

)"
