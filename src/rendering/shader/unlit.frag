R"(
#version 450 core

layout(location = 0) out vec4 color;

layout(location = 1) uniform vec4 color_in;

void main() {
	color = color_in;
}
)"
