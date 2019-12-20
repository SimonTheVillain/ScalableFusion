R"(
#version 450 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout(location = 0) out vec4 color;

flat in int tri_ind;
flat in int vert_ind1;
flat in int vert_ind2;
flat in int vert_ind3;
in vec3 barycentric_weights;//this is not (barycentric interpolation?)

void main(void) {
	color = vec4(1, 0, 0, 1);

	//storing triangle index
	color.x = intBitsToFloat(tri_ind);
	// + barycentric weights
	color.yzw = barycentric_weights;

	//TODO: store 16 bit triangle index + 2x 8 bit barycentric weight (quarter of the memory footprint)



	//Implementation that stored 3 vertex indices + 3 barycentric weights as bytes. ( not ideal)
	/*
	color.x = intBitsToFloat(vert_ind1);
	color.y = intBitsToFloat(vert_ind2);
	color.t = intBitsToFloat(vert_ind3);

	//store the barycentric coordinates as uint 8 into an 32 bit uint;
	uint bary = uint(barycentric_weights.x * 255.0) << 24;
	bary += uint(barycentric_weights.y * 255.0) << 16;
	bary += uint(barycentric_weights.z * 255.0) << 8;
	color.w = uintBitsToFloat(bary);
	*/
	//TODO: alternatively we could store the indices as 16 bit variables and only 2 of the 3 barycentric coordinates
	//this would only require 2 floats/  4 halfs
}
)"
