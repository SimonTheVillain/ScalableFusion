R"(
#version 450 core

layout(location = 0) out vec4 tex_ref;
layout(location = 1) out vec4 geometry;
in vec4 color_in;
in vec4 normal_in;
in vec4 geometry_in;

void main() {
	tex_ref = vec4(0, 0, 0, 10.0f);
	geometry = geometry_in;
}
)"
