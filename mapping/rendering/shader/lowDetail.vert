R"(
#version 450 core

layout(location = 0) in vec4 pos;
layout(location = 1) in vec4 normal;
layout(location = 2) in vec4 color;
layout(location = 3) in int valid;


out vec4 normals;
out vec4 colors;
out int valids;

layout(location = 0) uniform mat4 proj_rans;

void main() {
	gl_Position = proj_trans * pos;
	normals = normal; //actually this needs to be transformed
	valids  = valid;
	colors  = color;
}
)"
