R"(
#version 450 core

layout(location = 0) out vec4 color;
in vec4 color_in;
in vec4 normal_in;

void main() {
    color = color_in;
}
)"
