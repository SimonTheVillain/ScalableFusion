R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

layout(location = 0) out vec4 color;

void main(void) {
    color = vec4(1, 0, 0, 1);
}
)"
