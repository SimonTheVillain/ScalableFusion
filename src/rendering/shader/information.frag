R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 position;
layout(location = 1) out vec4 color;

//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs

//this location is asking for trouble!!!!!!
//these should be in buffers not uniforms or even attributes??
layout(location = 4) uniform mat4 proj_matrix_;
//use the inverted projection matrix to restore the unprojected point coordinates.

in vec2 tex_pos_out;
in vec4 interp_position;//TODO!!!! interpolation like this is not the right for geometry
in vec4 interp_proj;//debug is this the same as gl_FragCoord
flat in uint64_t bindless_texture;
flat in int is_stitch;
in float z;
void main(void) {
    position = proj_matrix_ * interp_proj;
    //bindless texture code:
    sampler2D s = sampler2D(bindless_texture);
    color = texture(s, tex_pos_out); //scale the texPos to see a more dramatic effect
    //TODO: The red dots in the debug window seems to show that the geometric texture
    //gets sampled where the according lookup table was not written. (thus it doesn't get updated
    // where it should be)

    color.x = z;
    color.x = color.x;
}
)"
