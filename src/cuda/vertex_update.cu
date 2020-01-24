#include <cuda/vertex_update.h>

#include <cuda/surface_read.h>
#include <cuda/xtion_camera_model.h>
#include <cuda/float16_utils.h>
#include <gpu/gpu_mesh_structure.h>

using namespace Eigen;

__global__ 
void vertexUpdate_kernel(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                         int sensor_width,int sensor_height, //sensor resolution
                         gpu::UpdateDescriptor *descriptors,
                         Vector4f cam_pos,//camera position
                         Matrix4f pose, // because we want the vertex position relative to the camera
                         Matrix4f proj_pose) { //to get the position of the point on the image.


	uint32_t k = blockIdx.x;
	uint32_t i = threadIdx.x;

	gpu::UpdateDescriptor &desc = descriptors[k];
	//GpuPatchInfo &info = patch_infos[descriptor.patch_info_slot];
	//uint32_t vertex_source_offset = descriptor.vertex_source_start_ind;
	//uint32_t vertex_dest_offset = descriptor.vertex_destination_start_ind;
	
	//uint32_t tex_pos_offset = info.std_texture.tex_coord_start_ind;
	if(i==0){
		//printf("0x%p 0x%p %d\n",desc.src_verts, desc.dst_verts,desc.vertex_count);
	}
	while(i < desc.vertex_count) {
		GpuVertex &vert_in = desc.src_verts[i];
		GpuVertex &vert_out = desc.dst_verts[i];

		//most of the vertex is the same anyway (therefore we just copy it)
		vert_out = vert_in;

		//DEBUG:
		if(false){
			i += blockDim.x;
			continue;
		}
		Vector4f pos = vert_in.p;
		pos[3] = 1; //just to be sure;


		Vector4f pos_cam_frame = pose * pos;
		Vector4f pos_on_sensor = proj_pose * pos;

		float u = pos_on_sensor[0] / pos_on_sensor[3];
		float v = pos_on_sensor[1] / pos_on_sensor[3];

		//TODO: generate threshold here:

		float4 sensor = readSensor(u, v, geometry_input, sensor_width, 
		                           sensor_height, 0.05);//threshold =0.1
		//printf("u %f v %f \n",u,v);
		//printf("sensor data %f %f %f %f \n", sensor.x,sensor.y,sensor.z,sensor.w);
		//DEBUG:
		if(false){
			i += blockDim.x;
			continue;
		}
		//extract the depth from the readout data
		float d = pos_cam_frame[2];
		float d_s = sensor.x;

		//get the camera position relative to our vertex
		Vector4f to_camera = cam_pos - pos;
		Vector3f to_cam_n = to_camera.block<3, 1>(0, 0) * 1.0f /
		                    to_camera.block<3, 1>(0, 0).norm();
		Vector4f front(to_cam_n[0], to_cam_n[1], to_cam_n[2], 0);

		//read the current value of the texture
		Vector2f tex_coord = desc.src_tex_coords[i];

		//this tex coordinate still has to be adapted for the texture atlas
		float2 tex_atlas_coord = make_float2(
				tex_coord[0] * desc.source_n.width  + desc.source_n.x,
				tex_coord[1] * desc.source_n.height + desc.source_n.y);


		//printf("uv.x %f uv.y %f \n",tex_coord[0],tex_coord[1]);
		//DEBUG:
		if(false){
			i += blockDim.x;
			continue;
		}

		/*
=========     at 0x000006b0 in vertexUpdate_kernel(unsigned __int64, int, int, gpu::UpdateDescriptor*, Eigen::Matrix<float, int=4, int=1, int=0, int=4, int=1>, Eigen::Matrix<float, int=4, int=4, int=0, int=4, int=4>, Eigen::Matrix<float, int=4, int=4, int=0, int=4, int=4>, GpuVertex*, Eigen::Matrix<float, int=2, int=1, int=0, int=2, int=1>*, GpuTriangle*, GpuPatchInfo*)
=========     by thread (16,0,0) in block (9,0,0)
=========     Address 0x00000080 is out of bounds
=========     Saved host backtrace up to driver entry
		 */
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
										 desc.source);

		//printf("uv.x %f uv.y %f \n",uv.x,uv.y);
		//DEBUG:
		if(false){
			i += blockDim.x;
			continue;
		}


		float4 surface_k;
		{
			vert_out.n = Vector3f(0, 0, 1);
			__half surface_data[4];
			surf2Dread((ushort4*) surface_data, desc.source_geometry,
			           int(uv.x) * sizeof(ushort4), int(uv.y));
			surface_k = make_float4(__half2float(surface_data[0]),
			                        __half2float(surface_data[1]),
			                        __half2float(surface_data[2]),
			                        __half2float(surface_data[3]));
			//debug: i really think it is the readout here!!!!!
		}

		vert_out.n = Vector3f(0, 1, 0);

		//printf("uv.x %f uv.y %f \n",uv.x,uv.y);
		//printf("sensor data %f %f %f %f \n", surface_k.x,surface_k.y,surface_k.z,surface_k.w);

		//DEBUG:
		if(false){
			i += blockDim.x;
			continue;
		}

		//DEBUG: not crashing till here! (REMOVE COMMENT)
		if(isnan(surface_k.w)) {//actually this should be 1
			//if this surface element is invalid we store the sensor input at the position that is lacking

			float4 update = make_float4(0, sensor.y, sensor.z, 2);//mark the lonesome point as being a lonesome point (2)

			ushort4 surface_data = float4_2_half4_reinterpret_ushort4_rn(update);

			//printf("uv.x %f uv.y %f \n",uv.x,uv.y);
			//TODO: reinsert this

			/*
			surf2Dwrite(surface_data, desc.source_geometry,
			            int(uv.x) * sizeof(ushort4), int(uv.y));
			*/
			//TODO: Important!!! Make sure that also the reference is set properly (otherwise we will not see an update)

			if(int(uv.x) < desc.source.x ||
			   int(uv.x) - desc.source.x >= desc.source.width ||
			   int(uv.y) < desc.source.y ||
			   int(uv.y) - desc.source.y >= desc.source.height) {
				vert_out.n = Vector3f(0, 1, 0);// debug
			} else {
				vert_out.n = Vector3f(1, 0, 0);// debug
			}
			//obviously this vertex had no correct pixel yet therefore we set a pixel in there
			i += blockDim.x;
			continue;
		}
		//DEBUG:
		if(false){
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

		//DEBUG:
		if(false){
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
			float d_up = d - dev_k1;
			threshold = xtionStdToThresholdSeg(sensor.z);
			if(abs(d_up) < threshold) {
				break;
			}
			//update the standard deviation in the same way as it would be done
			float4 update = calcSurfaceUpdate(surface_k, sensor, d, d_up);
			
			update.x = 0;
			ushort4 surface_data = float4_2_half4_reinterpret_ushort4_rn(update);
			surf2Dwrite(surface_data, desc.destination_geometry,
			            int(uv.x) * sizeof(ushort4), int(uv.y));

			//TODO: make sure to at some point update the normals by sensible means
			vert_out.n = Vector3f(0, 1, 0);
		}

		if(int(uv.x) < desc.source.x ||
		   int(uv.x) - desc.source.x >= desc.source.width ||
		   int(uv.y) < desc.source.y ||
		   int(uv.y) - desc.source.y >= desc.source.height) {
			//set the normal to a debug color!
			vert_out.n = Vector3f(1, 1, 1);
		}

		//just in case we have more vertices than threads:
		i += blockDim.x;
	}
}


__global__
void transcribe_stitch_vertices_kernel(gpu::GeometryUpdate::TranscribeStitchTask* tasks){//pointer chasing deluxe!
	uint32_t k = blockIdx.x;
	uint32_t i = threadIdx.x;
	gpu::GeometryUpdate::TranscribeStitchTask &task = tasks[k];
	if(i == 0){
		//printf("task.count%d \n",task.count);
	}
	while(i<task.count){
		//check for the destination:
		//Vector4f p = task.local_vertices[task.task[i].ind_local].p;
		//printf("task.task[i].ind_local %d \n p %f %f %f %f \n",task.task[i].ind_local,p[0],p[1],p[2],p[3]);

		//check for the source



		task.local_vertices[task.task[i].ind_local] =
				task.nb_vertices[task.task[i].ind_neighbour][task.task[i].ind_in_neighbour];


		i+=blockDim.x;
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