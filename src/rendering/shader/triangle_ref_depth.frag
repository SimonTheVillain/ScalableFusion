R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 tex_ref;//triangle reference
layout(location = 1) out vec4 geometry;

//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs

flat in int is_stitch;//maybe later
flat in int patch_id;
flat in int triangle_index;

in vec4 point_world;

void main(void) {
	tex_ref.z = 2;
	tex_ref.w = 3.0;
	tex_ref.x = intBitsToFloat(patch_id);
	tex_ref.y = intBitsToFloat(triangle_index);
	geometry  = point_world;
}
)"
