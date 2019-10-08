#include "stdTexUpdate.h"

#include <assert.h>

#include "gpuErrchk.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"
#include "float16_utils.h"

using namespace std;
using namespace Eigen;

__device__ 
inline void writeResult(float4 result, cudaSurfaceObject_t surface, 
                        int x, int y) {
	ushort4 output = float4_2_half4_reinterpret_ushort4_rn(result);
	surf2Dwrite(output, surface, x * sizeof(ushort4), y);
}

__global__ 
void updateGeomTex_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                          int width,int height, //sensor resolution
                          gpu::UpdateDescriptor *descriptors,
                          Vector4f cam_pos,//camera position
                          Matrix4f pose, // because we want the vertex position relative to the camera
                          Matrix4f proj_pose, //to get the position of the point on the image.
                          GpuVertex *vertices, Vector2f *tex_pos,
                          GpuTriangle* triangles,GpuPatchInfo* patch_infos) {

	const int k = blockIdx.x;
	gpu::UpdateDescriptor &descriptor = descriptors[k];

	int i = threadIdx.x;
	uint32_t vertex_source_offset = descriptor.vertex_source_start_ind;
	uint32_t vertex_dest_offset = descriptor.vertex_destination_start_ind;

	int absolute_pix_count = 
			descriptor.destination.height * descriptor.destination.width;
	while(i < absolute_pix_count) {
		int x = i % descriptor.destination.width;
		int y = i / descriptor.destination.width;

		//get the pixel coordinate (not normalized)
		int x_dest = x + descriptor.destination.x;
		int y_dest = y + descriptor.destination.y;

		int x_ref = x + descriptor.reference_offset.x;
		int y_ref = y + descriptor.reference_offset.y;

		//the propable source coordinate (not normalized)
		float x_source = 
				(x * descriptor.source.width) / descriptor.destination.width + descriptor.source.x;
		float y_source = 
				(y * descriptor.source.height) / descriptor.destination.height + descriptor.source.y;

		//read from the reference texture
		float4 ref;
		surf2Dread(&ref, descriptor.destination_references, x_ref*sizeof(Vector4f),
		           y_ref);

		//TODO: if the  vertex doesn't have a triangle within this patch
		//we want to use a different method. (but for this to work we need to
		//implement code in the vertex update part)

		float bary[3] = {ref.y, ref.z, ref.w};
		int32_t triangle_id = *((int*) (&ref.x));

		if(triangle_id < 0) {
			i += blockDim.x;
			continue;
		}
		GpuTriangle &triangle = triangles[triangle_id];

		//read out the vertices
		Vector4f point(0, 0, 0, 0);
		Vector4f point_updated(0, 0, 0, 0);
		for(int j = 0; j < 3; j++) {
			point += vertices[vertex_source_offset + triangle.indices[j]].p * bary[j];
			point_updated += 
					vertices[vertex_dest_offset + triangle.indices[j]].p * bary[j];
		}

		//project:
		Vector4f tex_coord = proj_pose * point;
		float u = tex_coord[0] / tex_coord[3];
		float v = tex_coord[1] / tex_coord[3];

		//read out the current sensor data
		//TODO: this readout probably is shit.....
		float4 sensor = readBilinear(u, v, geometry_input, width, height);

		//this should be working better
		sensor = readSensor(u, v, geometry_input, width, height, 0.05f);//5cm threshold

		float4 surface_k;
		if(descriptor.destination.width == descriptor.source.width &&
		   descriptor.destination.height == descriptor.source.height) {
			//if the source and the destRect
			//texture have the same resolution we handle all of this differently
			surface_k = readFloat4F16(descriptor.source_geometry, x_source, y_source);

		} else {
			surface_k = readBilinear16F(x_source, y_source, 
			                            descriptor.source_geometry,
			                            descriptor.source_size.width,
			                            descriptor.source_size.height);
		}

		//hopefully the border for this readout will be NON
		if(isnan(surface_k.y)) {
			//if the old surface data is invalid we just put out the
			//sensor data. (hoping that this is better than nothing)
			//actually this should have a visibility test (TODO)
			writeResult(make_float4(0, sensor.y, sensor.z, sensor.w),
			            descriptor.destination_geometry,
			            x_dest, y_dest);
			i += blockDim.x;
			continue;
		}
		if(isnan(sensor.y)) {
			//the sensor doesn't have any valid values
			//keep the old surface parameter
			writeResult(surface_k, descriptor.destination_geometry, x_dest, y_dest);
			i += blockDim.x;
			continue;
		}

		//transform the coordinates relative to camera
		Vector4f pos_cam_frame = pose*point;

		Vector4f pos_up_cam_frame = pose * point_updated;
		//depth test against sensor input
		float d    = pos_cam_frame[2];
		float d_up = pos_up_cam_frame[2];

		//TODO: make this threshold dependant on something
		float threshold = 0.05f;//+- 5 cm? maybe this is not the right one.

		threshold = xtionStdToThresholdSeg(max(surface_k.z, sensor.z));
		//(like the standard deviation of the sensor)
		if(abs(d - sensor.x) > threshold) {
			//if the surface is hidden by some other surface.
			//keep the old surface parameter
			writeResult(surface_k, descriptor.destination_geometry, x_dest, y_dest);
			i += blockDim.x;
			continue;
		}

		float4 surface_k1 = calcSurfaceUpdate(surface_k, sensor, //the vector of sensor data and of what is on the surface
		                                      d, d_up);
		writeResult(surface_k1, descriptor.destination_geometry, x_dest, y_dest);

		i += blockDim.x;
	}

	//done!!!!!!


	//TO think
	//NO!!!!!it is not! we can't do the update step upfront.
	//but for each point we can store the position of how it was before and how it will be afterwards.....
	//so we can have the information of each point before and after the update.
	//thus we can calculate for each pixel the distane between the old and the new surface.
	//ONE QUESTION REMAINS: SHOULD THIS BE DONE IN ONE OR TWO KERNEL CALLS????
	//for 2 kernel calls we would need a buffer to store the intermediate results

	//For 1 kernel call we have the problem of some of the vertices not being part of the
	//patch. (actually this problem applies for both approaches)

	//We need 2 calls + one copy of the vertex buffer!!!!:
	//1) update the new texture + update the second vertex buffer
	//2) update the change in distance between the geometry gathered by the second
	//   vertex buffer compared to the first write the geometry from the second buffer
	//   to the first.


	//TE DO THIS:!!!!!!
	//without only one kernel call:
	// First: we create the updated version of the vertices in the shared memory
	// second:
	// for each pixel we read the 3 neighboring vertices.
	// we create the updated version of the vertex + keep the old one,
	// according to this we update the texture. ( this means calculating the update
	// according to the old geometry and later updating it with the new one)
	// After writing out the texture we write out the vertices.

}

void updateGeomTexturesOfPatches(
		const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
    int width, int height, //sensor resolution
    const vector<gpu::UpdateDescriptor> &descriptors,
    Vector4f cam_pos,
    Matrix4f pose, // because we want the vertex position relative to the camera
    Matrix4f proj_pose, //to get the position of the point on the image.
    GpuVertex *vertices, Vector2f *tex_pos,
    GpuTriangle *triangles,GpuPatchInfo *patch_infos) { //pointer to the geometric data

	if(descriptors.empty()) {
		return;
	}

	dim3 block(1024);
	dim3 grid(descriptors.size());
	gpu::UpdateDescriptor *descs;
	cudaMalloc(&descs, descriptors.size() * sizeof(gpu::UpdateDescriptor));
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(gpu::UpdateDescriptor),
	           cudaMemcpyHostToDevice);

	gpuErrchk(cudaPeekAtLastError());

	updateGeomTex_kernel<<<grid, block>>>(geometry_input, //the sensor input adapted by standard deviations
	                                      width, height, //sensor resolution
	                                      descs,
	                                      cam_pos,
	                                      pose, // because we want the vertex position relative to the camera
	                                      proj_pose, //to get the position of the point on the image.
	                                      vertices, tex_pos,
	                                      triangles, patch_infos);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	cudaFree(descs);
}

__global__ 
void dilateLookupTextures_kernel(const DilationDescriptor* descriptors) {
	unsigned int k = blockIdx.x;
	const DilationDescriptor &descriptor = descriptors[k];
	const unsigned int required_pixel = descriptor.height * descriptor.width;
	unsigned int i = threadIdx.x;

	const int2 direct[] = {make_int2(-1, 0), make_int2(1, 0), make_int2(0, -1), 
	                       make_int2(0,1)};
	const int2 indirect[] = {make_int2(-1, -1), make_int2(1, 1), make_int2(1, -1),
	                         make_int2(-1,1)};

	while(i < required_pixel) {
		int x = i % descriptor.width + descriptor.x;//derive the target pixel coordinates from k
		int y = i / descriptor.width + descriptor.y;
		float4 center;
		surf2Dread(&center, descriptor.target, x * 4 * 4, y);
		int index = *((int*) (&center.x));

		if(index != -1){ //if the triangle index already is valid we don't do this
			i += blockDim.x;
			continue;
		}

		bool set = false;
		//otherwise we search at the neighbouring locations
		for(int j = 0; j < 4; j++) {
			int x2 = x + direct[j].x;
			int y2 = y + direct[j].y;
			if(x2 < descriptor.x || 
			   y2 < descriptor.y ||
			   x2 >= (descriptor.width + descriptor.x) ||
			   y2 >= (descriptor.height + descriptor.y)) {
				continue;
			}
			float4 neighbour;
			surf2Dread(&neighbour, descriptor.target, x2 * 4 * 4, y2);
			index = *((int*) (&neighbour.x));
			if(index == -1) {
				continue;
			}
			surf2Dwrite(neighbour, descriptor.target, x * 4 * 4, y);
			set = true;
			break;
		}
		if(set) {
			i += blockDim.x;
			continue;
		}
		//if this didn't yield results we do it diagonally
		for(int j = 0; j < 4; j++) {
			int x2 = x + indirect[j].x;
			int y2 = y + indirect[j].y;
			if(x2 < descriptor.x || 
			   y2 < descriptor.y ||
			   x2 >= (descriptor.width + descriptor.x) ||
			   y2 >= (descriptor.height + descriptor.y)) {
				continue;
			}
			float4 neighbour;
			surf2Dread(&neighbour, descriptor.target, x2 * 4 * 4, y2);
			index = *((int*) (&neighbour.x));
			if(index == -1){
				continue;
			}
			surf2Dwrite(neighbour, descriptor.target, x * 4 * 4, y);
			set = true;
			break;
		}
		//do the next few pixel
		i += blockDim.x;
	}
}

void dilateLookupTextures(const vector<DilationDescriptor> &descriptors) {
	if(descriptors.empty()) {
		return;
	}

	dim3 block(1024);
	dim3 grid(descriptors.size());

	DilationDescriptor *descs;
	cudaMalloc(&descs, descriptors.size() * sizeof(DilationDescriptor));
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(DilationDescriptor), 
	           cudaMemcpyHostToDevice);

	dilateLookupTextures_kernel<<<grid, block>>>(descs);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	cudaFree(descs);
}

__global__ 
void stdTexInit_kernel(const cudaTextureObject_t input,
                       const InitDescriptor *descriptors,
                       Matrix4f proj_pose,
                       GpuVertex *vertices,
                       Vector2f *tex_pos,
                       GpuTriangle *triangles, GpuPatchInfo *patch_infos) {

	unsigned int k = blockIdx.x;
	const InitDescriptor &descriptor = descriptors[k];
	const unsigned int required_pixel = descriptor.height * descriptor.width;
	unsigned int i = threadIdx.x;

	while(i < required_pixel) {
		int x_ref = i % descriptor.width + descriptor.ref_offset.x;//derive the target pixel coordinates from k
		int y_ref = i / descriptor.width + descriptor.ref_offset.y;
		int x_out = i % descriptor.width + descriptor.out_offset.x;//derive the target pixel coordinates from k
		int y_out = i / descriptor.width + descriptor.out_offset.y;

		float4 ref;
		surf2Dread(&ref, descriptor.reference_texture, x_ref * 4 * 4, y_ref);
		float bary[3] = {ref.y, ref.z, ref.w};
		int32_t triangle_id = *((int*) (&ref.x));//reinterpret the float as integer.... i really hope this works

		if(triangle_id < 0) {
			float4 color = make_float4(NAN, NAN, NAN, NAN);

			ushort4 data = float4_2_half4_reinterpret_ushort4_rn(color);
			surf2Dwrite(data, descriptor.output, x_out * sizeof(ushort4), y_out);//write some debug value to the output (seems to work)

			i += blockDim.x;
			continue;
		}

		GpuTriangle triangle = triangles[triangle_id];
		Vector4f point(0, 0, 0, 0);
		for(int j = 0; j < 3; j++) {
			GpuPatchInfo &info = patch_infos[triangle.patch_info_inds[j]];
			int index = info.vertex_source_start_ind + triangle.indices[j];
			point += vertices[index].p * bary[j];
		}

		Vector4f tex_coord = proj_pose * point;//project (Non-normalized coordinates)
		//TODO: depth test and reading of correct color/geom values
		//are these normal values
		float u = tex_coord[0] / tex_coord[3];
		float v = tex_coord[1] / tex_coord[3];

		//look at applyNewColorData in scaleableMapTexturing, there the creation of texture
		//coordinates works well. why isn't it working here?

		//if this is right do the projection and read the according values. (also do a depth test)

		float4 color = make_float4(point[0], point[1], point[2], point[3]);
		color = make_float4(u, v, 0.0f, 1.0f);
		float4 sensor = tex2D<float4>(input, u, v);

		//TODO: keep the standard deviations 1/1 but set the depth offset to zero, when there are NANs we
		//set the values to the closest possible value
		color.x = 0;//depth offset is zero,
		color.y = sensor.y; // here we have to check if it is NAN... this would not be too awesome
		color.z = sensor.z; // same here, i don't know what to do tough

		ushort4 output = float4_2_half4_reinterpret_ushort4_rn(color);

		surf2Dwrite(output, descriptor.output, x_out * sizeof(ushort4), y_out);

		//do the next block
		i += blockDim.x;
	}
}

void stdTexInit(const cudaTextureObject_t input,
               const vector<InitDescriptor> &descriptors,
               Matrix4f proj_pose,
               GpuVertex *vertices, Vector2f *tex_pos,
               GpuTriangle *triangles, GpuPatchInfo *patch_infos){

	InitDescriptor *descs = nullptr;
	//create a buffer for all the commands:
	//TODO: DELETE THIS DEBUG SHIT!!!!!
	for(size_t i = 0; i < descriptors.size(); i++) {
		InitDescriptor desc = descriptors[i];
	}
	cudaMalloc(&descs, descriptors.size() * sizeof(InitDescriptor));

	gpuErrchk(cudaPeekAtLastError());
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(InitDescriptor),
	           cudaMemcpyHostToDevice);

	gpuErrchk(cudaPeekAtLastError());

	dim3 block(1024);//smaller is better for smaller images. how is it with bigger images?
	//the ideal way would be to start blocks of different sizes. (or maybe use CUDAs dynamic parallelism)
	//for higher resolution images 1024 or even higher threadcounts might be useful
	dim3 grid(descriptors.size());
	if(grid.x != 0) {
		stdTexInit_kernel<<<grid, block>>>(input, descs, proj_pose, vertices,
		                                   tex_pos, triangles, patch_infos);
	}
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	//AS LONG AS WE ARE NOT REUPLOADING AND CREATING NEW REFERENCE TEXTURES TO THE GEOMETRY
	//THIS JUST FAILS (HOPEFULLY,  BECAUSE THE REFERENCES ARE POINTING TO NOWHERE)

	cudaFree(descs);
}




//this was something i did for debugging purposes. it is supposed to shift the
__global__ 
void shiftVerticesALittle_kernel(GpuVertex *vertices, size_t from, size_t to) {
	size_t element = threadIdx.x + blockIdx.x * blockDim.x + from;
	if(element >= to) {
		return;//return if we are exceeding the buffer
	}
	Vector4f offset(10.1, 0, 0, 0);
	vertices[element].p += offset; //that should be it
}

void shiftVerticesALittle(GpuVertex *vertices, size_t from, size_t to) {
	size_t vertex_count = to - from;
	dim3 block(256);
	dim3 grid(vertex_count / block.x + 1);

	shiftVerticesALittle_kernel<<<block, grid>>>(vertices, from, to);
	cudaDeviceSynchronize();
}
