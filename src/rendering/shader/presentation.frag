R"(
#version 460 core

#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5 : require // for uint64_t

//i think it is key to use bindless textures for all this shit!!!!
//http://www.geeks3d.com/20120511/nvidia-gtx-680-opengl-bindless-textures-demo/

//this guy is awesome:
//https://www.youtube.com/watch?v=-bCeNzgiJ8I
layout (location = 0) out vec4 color;

//this guy seems to have a good approach to uniforms:
//https://www.opengl.org/discussion_boards/showthread.php/177593-Nvidia-bindless-textures-first-experiences-and-bugs

in vec2 tex_pos_out;
in vec4 interp_position;//TODO!!!! interpolation like this is not the right for geometry
in vec4 interp_proj;//debug is this the same as gl_FragCoord
in vec4 normal;
flat in uint64_t bindless_texture;
flat in int is_stitch;
flat in int patch_id;
flat in int tex_coord_slot_out;//thats a debug thingy
in float z;

layout (location = 4) uniform int  render_wireframe;
layout (location = 5) uniform int  color_mode;
layout (location = 6) uniform int  lighting_mode;
layout (location = 7) uniform vec4 line_color;

//this is needed for
in float distances[3];
flat in int32_t debug1;
float line_width = 0.9;

flat in int32_t debug2;

vec4 colorCode(int32_t id) {
	uint8_t r = uint8_t((id / 10 * 50) % 255);
	uint8_t g = uint8_t((id % 7) * 30);
	uint8_t b = uint8_t((id % 5) * 50);
	return vec4(float(b) / 255.0, float(g) / 255.0, float(r) / 255.0, 1.0);
}

void main(void) {

	/*
	if(debug2 == 1) {
		color = vec4(0.7, 0.7, 0,1);
		return;
	}

	if(debug1 == 1) {
		color = vec4(0, 1, 0, 1);
		return;
	}

	*/
	if(bindless_texture == 0) {
		//if there is no texture bound we put out a other color
		color = vec4(0.5, 0.5, 0, 1);
	//	return;
	} else {
		sampler2D s = sampler2D(bindless_texture);
		color = texture(s, tex_pos_out);
	}

	//rendering the wireframe
	float min_distance = min(distances[0], min(distances[1], distances[2]));
	float mix_val = smoothstep(line_width - 0.5, line_width + 0.5, min_distance);





	if(color_mode == 1) {
		color = colorCode(patch_id);
	} else if(color_mode == 2) {
		//presenting the standard deviation textures....
		//multiplying with 10 is a good idea there
		color.xyz = color.xyz * 10.0;
		//color.xy = tex_pos_out;//debug!!!
		//color.zw = vec2(0,1);
	} else if(color_mode == 3) {
		color = -normal * 0.5 + vec4(0.5, 0.5, 0.5, 0.5);
		color.w = 1.0;
	} else if(color_mode == 4) {
		if(floatBitsToInt(color.x) < 0) {
			color.xyz = vec3(1, 0, 0);
		} else {
			color.xyz = color.yzw;
		}
		color.w = 1.0;
	} else if(color_mode == 5) {
		if(true) {
			if(floatBitsToInt(color.x) < 0) {
				color.xyz = vec3(1, 0, 0);
			} else {
				color.xyz = color.yzw;
			}
		}

		return;//debug: show the raw texture
		int segment_label = floatBitsToInt(color.x);
		color = vec4(color.x, 0, 0, 1);//another debug
		color = vec4(tex_pos_out, 0, 1);//why is this flickering? why!!!!!!
	}

	//TODO: completely remove lighting modes
	/*
	//certain debug outputs
	if(is_stitch != 0) {
	}

	if(tex_coord_slot_out == 10) {
	}

	if(lighting_mode == 0) {
		//leave it as it is
	}

	if(lighting_mode == 1) {
		//flat shading
		//debug
		color = normal * 0.5 + vec4(0.5, 0.5, 0.5, 0.5);
		color.w = 1;
	}

	if(lighting_mode == 2) {
		//phong shading
	}

	if(lighting_mode == 3) {
		color = vec4(tex_pos_out, 0, 1);//debug
	}
	*/
	//color = colorCode(patch_id);
	if(render_wireframe != 0) {
		color = mix(line_color, color, mix_val);
	}
}
)"
