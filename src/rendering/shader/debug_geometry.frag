R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

layout(location = 0) out vec4 color_out;

layout(location = 2) uniform vec4 color;

in float distances[3];
float line_width = 0.9;
vec4  line_color  = vec4(0,0,0,1);


void main(void) {
    color_out = color;

    float min_distance = min(distances[0], min(distances[1], distances[2]));
    float mix_val = smoothstep(line_width - 0.5, line_width + 0.5, min_distance);
    color_out = mix(line_color, color, mix_val);
}
)"
