#include "vertexUpdate.h"

#include <iostream>
#include <stdio.h>
#include <assert.h>

#include "helper_math.h"
#include "surfaceRead.h"
#include "xtionCameraModel.h"
#include "geomUpdate.h"
#include "float16_utils.h"

using namespace Eigen;

__global__ 
void vertexUpdate_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                         int sensor_width,int sensor_height, //sensor resolution
                         gpu::UpdateDescriptor *descriptors,
                         Vector4f cam_pos,//camera position
                         Matrix4f pose, // because we want the vertex position relative to the camera
                         Matrix4f proj_pose, //to get the position of the point on the image.
                         GpuVertex *vertices, Vector2f *tex_pos,
                         GpuTriangle *triangles, GpuPatchInfo* patch_infos) {

	uint32_t k = blockIdx.x;
	uint32_t i = threadIdx.x;

	gpu::UpdateDescriptor &descriptor = descriptors[k];
	GpuPatchInfo &info = patch_infos[descriptor.patch_info_slot];
	uint32_t vertex_source_offset = descriptor.vertex_source_start_ind;
	uint32_t vertex_dest_offset = descriptor.vertex_destination_start_ind;
	
	uint32_t tex_pos_offset = info.std_texture.tex_coord_start_ind;
	while(i < descriptor.vertex_count) {
		GpuVertex &vert_in = vertices[vertex_source_offset + i];
		GpuVertex &vert_out = vertices[vertex_dest_offset + i];

		//most of the vertex is the same anyway (therefore we just copy it)
		vert_out = vert_in;

		Vector4f pos = vert_in.p;
		pos[3] = 1; //just to be sure;


		Vector4f pos_cam_frame = pose * pos;
		Vector4f pos_on_sensor = proj_pose * pos;

		float u = pos_on_sensor[0] / pos_on_sensor[3];
		float v = pos_on_sensor[1] / pos_on_sensor[3];

		//TODO: generate threshold here:

		float4 sensor = readSensor(u, v, geometry_input, sensor_width, 
		                           sensor_height, 0.05);//threshold =0.1

		//extract the depth from the readout data
		float d = pos_cam_frame[2];
		float d_s = sensor.x;

		//get the camera position relative to our vertex
		Vector4f to_camera = cam_pos - pos;
		Vector3f to_cam_n = to_camera.block<3, 1>(0, 0) * 1.0f /
		                    to_camera.block<3, 1>(0, 0).norm();
		Vector4f front(to_cam_n[0], to_cam_n[1], to_cam_n[2], 0);

		//read the current value of the texture
		Vector2f tex_coord = tex_pos[tex_pos_offset + vert_in.tex_ind_in_main_patch];

		//this tex coordinate still has to be adapted for the texture atlas
		float2 tex_atlas_coord = make_float2(
				tex_coord[0] * descriptor.source_n.width + descriptor.source_n.x,
				tex_coord[1] * descriptor.source_n.height + descriptor.source_n.y);

		//TODO: we need a function that explicitely handles reading textures from this type of texture

		//texture atlas
		//PLUS: if the point we want to read out has no valid pixel (not set for some reason)
		//we set the pixel at this position. (in the next readout we might be lucky)
		//reading the source geometry
		/*float4 surface_k=
				readBilinearGLStyle(texAtlasCoord.x,texAtlasCoord.y,
									descriptor.sourceGeometry,
									descriptor.sourceSize.width,descriptor.sourceSize.height);
		*/
		float2 uv = unnormalizeTexCoords(make_float2(tex_coord[0], tex_coord[1]),
		                                             descriptor.source);
		float4 surface_k;
		{
			vert_out.n = Vector3f(0, 0, 1);
			__half surface_data[4];
			surf2Dread((ushort4*) surface_data, descriptor.source_geometry, 
			           int(uv.x) * sizeof(ushort4), int(uv.y));
			surface_k = make_float4(__half2float(surface_data[0]),
			                        __half2float(surface_data[1]),
			                        __half2float(surface_data[2]),
			                        __half2float(surface_data[3]));
			//debug: i really think it is the readout here!!!!!

		}

		vert_out.n = Vector3f(0, 1, 0);

		if(isnan(surface_k.w)) {//actually this should be 1
			//if this surface element is invalid we store the sensor input at the position that is lacking

			float4 update = make_float4(0, sensor.y, sensor.z, 2);//mark the lonesome point as being a lonesome point (2)

			ushort4 surface_data = float4_2_half4_reinterpret_ushort4_rn(update);

			//TODO: reinsert this
			surf2Dwrite(surface_data, descriptor.source_geometry, 
			            int(uv.x) * sizeof(ushort4), int(uv.y));

			//TODO: Important!!! Make sure that also the reference is set properly (otherwise we will not see an update)

			if(int(uv.x) < descriptor.source.x ||
			   int(uv.x) - descriptor.source.x >= descriptor.source.width ||
			   int(uv.y) < descriptor.source.y ||
			   int(uv.y) - descriptor.source.y >= descriptor.source.height) {
				vert_out.n = Vector3f(0, 1, 0);
			} else {
				vert_out.n = Vector3f(1, 0, 0);
			}
			//obviously this vertex had no correct pixel yet therefore we set a pixel in there
			i += blockDim.x;
			continue;
		}

		float threshold = xtionStdToThresholdSeg(max(sensor.z, surface_k.z));//the second one is the surface
		if(abs(d - d_s) > threshold) {
			//TODO: make these thresholds more dependant on the noise. (and or the normal)
			//if the depth is bigger than the depth of the sensor + some threshold
			//we do not continue due to being occluded.
			i += blockDim.x;
			continue;
		}

		#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
		if(k == 10)//i have the feeling that reading out from the surface is not working
			printf("surface parameters at vertex, %f, %f, %f, %f for index %d and tex coordinate %f, %f\n",
			       surface_k.x,surface_k.y,surface_k.z, surface_k.w,vertIn.texIndInMainPatch,
			       texAtlasCoord.x,texAtlasCoord.y);//texCoord[0],texCoord[1]);
		#endif

		float dev_s = d - d_s;//deviation of the sensor(d) from the surface (d_s)

		//the minimum possible standard deviation
		float s_m = min(sensor.z, surface_k.z);
		float s_k = surface_k.y;
		float s_s = sensor.y;

		//TODO: it actually would increase the numeric stability if the
		//texture would not store s_s but s_s_ so the distance to the minimal
		//possible standard deviation to prevent (numeric) cancellation.
		//standard deviation to the lower bound of standard deviation
		float s_s_ = s_s - s_m;
		float s_k_ = s_k - s_m;

		float dev_k1 = dev_s * s_k_ / (s_k_ + s_s_);
		if(isnan(dev_k1) || isinf(dev_k1)) {
			i += blockDim.x;
			continue;
		}

		//only the position changes
		vert_out.p = vert_in.p + dev_k1 * front;

		//if we are points that do not really belong to a triangle
		//we store the standard deviation within one pixel
		//the update also has to be done in here
		if(surface_k.w == 2.0f) {
			//TODO: think about depth dependant thresholding????
			float d_up= d - dev_k1;
			threshold = xtionStdToThresholdSeg(sensor.z);
			if(abs(d_up) < threshold) {
				break;
			}
			//update the standard deviation in the same way as it would be done
			float4 update = calcSurfaceUpdate(surface_k, sensor, d, d_up);
			
			update.x = 0;
			ushort4 surface_data = float4_2_half4_reinterpret_ushort4_rn(update);
			surf2Dwrite(surface_data, descriptor.destination_geometry, 
			            int(uv.x) * sizeof(ushort4), int(uv.y));

			//TODO: make sure to at some point update the normals by sensible means
			vert_out.n = Vector3f(0, 1, 0);
		}

		if(int(uv.x) < descriptor.source.x ||
		   int(uv.x) - descriptor.source.x >= descriptor.source.width ||
		   int(uv.y) < descriptor.source.y ||
		   int(uv.y) - descriptor.source.y >= descriptor.source.height) {
			//set the normal to a debug color?
			vert_out.n = Vector3f(1, 1, 1);
		}

		//just in case we have more vertices than threads:
		i += blockDim.x;
	}
}

__global__ 
void calcCenter_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks,
                       Vector4f *centers) {
	extern __shared__ Vector4f accu[];
	uint32_t k = blockIdx.x;
	gpu::GeometryUpdate::CalcCenterTask task = tasks[k];

	Vector3f accumulated(0, 0, 0);
	uint32_t i = threadIdx.x;

	while(i < task.count) {
		accumulated += task.vertices[i].p.block<3, 1>(0, 0);
		i += blockDim.x;
	}

	int tid = threadIdx.x;
	accu[tid].block<3, 1>(0, 0) = accumulated;
	__syncthreads();

	for(int s = blockDim.x / 2; s > 0; s >>= 1) {
		if(tid >= s) {
			//return;//TODO: maybe add this back in?!
			break;//just so we end up at syncthreads faster
		}
		if(tid < s) {
			accu[tid].block<3, 1>(0, 0) += accu[tid + s].block<3, 1>(0, 0);
		}
		__syncthreads();
	}

	if(tid == 0) {
		accu[0] = accu[0] * (1.0f / float(task.count));
	}

	__syncthreads();
	Vector3f center = accu[0].block<3, 1>(0, 0);

	//we can calc radii in the same kernel call!!!!!!

	float max_sqr_dist = 0;
	int max_ind = -1;
	i = threadIdx.x;
	while(i < task.count) {
		Vector3f pos = task.vertices[i].p.block<3, 1>(0, 0);
		Vector3f delta = pos - center;
		float sqr_distance = delta.dot(delta);

		if(sqr_distance>max_sqr_dist) {
			max_sqr_dist = sqr_distance;
			max_ind = i;
		}

		i += blockDim.x;
	}

	struct Outermost {
		int index;
		float sqr_distance;
	};
	//recasting the shared memory:
	Outermost *accuR = (Outermost*) accu;

	__syncthreads();//syncthreads here because we are using the same shared memory
	accuR[tid].index = max_ind;
	accuR[tid].sqr_distance = max_sqr_dist;
	__syncthreads();
	//reduction
	for(int s = blockDim.x / 2; s > 0; s>>= 1) {
		if(tid >= s) {
			return;//TODO: maybe add this back in?!
			break;//just so we end up at syncthreads faster
		}
		if(tid < s) {
			if(accuR[tid].sqr_distance < accuR[tid + 1].sqr_distance) {
			   accuR[tid].sqr_distance = accuR[tid + 1].sqr_distance;
			   accuR[tid].index = accuR[tid + 1].index;
			}
		}
		__syncthreads();
	}
	if(tid == 0) {
		centers[k] = Vector4f(center[0], center[1], center[2], 
		                      accuR[0].sqr_distance);
	}
}

__global__ 
void calcRadius_kernel(gpu::GeometryUpdate::CalcCenterTask *tasks, 
                       Vector4f *centers_radii) {

}