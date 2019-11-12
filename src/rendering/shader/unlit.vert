R"(
#version 450 core

layout(location = 0) in vec4 pos;

layout(location = 0) uniform mat4 proj_trans;

void main() {
	gl_Position = proj_trans * pos;
}
)"
