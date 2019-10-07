#include "labelling.h"

#include "gpuErrchk.h"
#include "surfaceRead.h"

using namespace std;
using namespace Eigen;

namespace gpu {

__global__ 
void labelSurfaces_kernel(Labelling::SegProjTask* tasks,
                          const cudaSurfaceObject_t labelling,
                          const cudaSurfaceObject_t geom_buffer,//the depth and geometry at the current view
                          cv::Size2i resolution,
                          Matrix4f _pose,
                          Matrix4f proj_pose,
                          GpuVertex *vertices,
                          Vector2f *tex_pos,
                          GpuTriangle *triangles,
                          GpuPatchInfo *patch_infos) {

	const int width = resolution.width;
	const int height = resolution.height;
	uint32_t k = blockIdx.x;
	Labelling::SegProjTask &task = tasks[k];
	uint32_t i = threadIdx.x;

	uint32_t vertex_source_offset = task.vertex_dest_start_ind;
	//uint32_t vertexDestOffset = task.vertexDestinationStartInd;

	int pixel_count = task.destination.width * task.destination.height;

	while(i < pixel_count) {
		// same procedure as in stdTexUpdate.cu
		int x = i % task.destination.width;
		int y = i / task.destination.width;

		int x_ref = x + task.lookup.x;
		int y_ref = y + task.lookup.y;

		int x_dest = x + task.destination.x;
		int y_dest = y + task.destination.y;

		//readout the lookup shizzle
		float4 ref;
		if(x_ref >= 1024 || y_ref >= 1024) {
			printf("This is not right!!!! %d, %d \n", x_ref, y_ref);
		}
		//TODO: find out why this is creating the memory exception
		surf2Dread(&ref, task.lookup_surf, x_ref * sizeof(Vector4f), y_ref);

		//get point in space shizzle:
		float bary[3] = {ref.y, ref.z, ref.w};
		int32_t triangle_id= *((int*) (&ref.x));

		if(triangle_id < 0) {
			i += blockDim.x;
			continue;
		}
		
		GpuTriangle &triangle = triangles[triangle_id];

		Vector4f point(0, 0, 0, 0);
		for(int j = 0; j < 3; j++) {
			point += vertices[vertex_source_offset + triangle.indices[j]].p * bary[j];
		}

		if(k == 0) {
			//debug
			Vector4f p = _pose * point;
		}

		//project point shizzle:
		Vector4f tex_coord = proj_pose * point;
		float u = tex_coord[0] / tex_coord[3];
		float v = tex_coord[1] / tex_coord[3];

		int ui = round(u);
		int vi = round(v);

		if(u < 0 || u > width || v < 0 || v > height || true) {
			//debug
		}

		//TODO: read old label if we already have a label!!!!!!!!
		int old_label;
		int4 old_label4;
		surf2Dread(&old_label4, task.dest_surf, x_dest * sizeof(int4), y_dest);
		old_label = old_label4.x;
		if(old_label != -1) {
			//TODO: when creating the label textures for the patches they need to be initialized with -1
			//don't overwrite the old label if it already exists
			i += blockDim.x;
			continue;
		}

		int new_label;
		if(ui < 0 || vi < 0 || ui >= width || vi >= height ||
		   u < 0 || v < 0 || u >= width || v >= height) {
			//this should actually barely happen!!!!!!
			//i+=blockDim.x;
			//continue;
			new_label = -1;
			new_label = -4;//debug
		} else {
			//we are not out of bounds
			float depth;
			surf2Dread(&depth, geom_buffer, ui * sizeof(float), vi);
			if(isnan(depth)) {
				new_label = -1;
				new_label = -2; // debug
			} else {
				float depth_threshold = 0.005f;//5mm should be enough (or we make it relative to the depth)
				Vector4f pos_cam_frame = _pose * point;
				if(fabs(pos_cam_frame[2] - depth) < depth_threshold) {
					new_label = -1;
					new_label = -3;//debug
				} else {
					//read the new label!!!!!
					surf2Dread(&new_label, labelling, ui * sizeof(int), vi);
					new_label = 100000;//debug
				}
			}
		}

		//write the label
		surf2Dwrite(new_label, task.dest_surf, 
		            x_dest * sizeof(Vector4i) + task.subchannel * sizeof(int),
		            //this could lead to alignment issues
		            y_dest);

		i += blockDim.x;
	}
}

void Labelling::labelSurfaces(vector<Labelling::SegProjTask> tasks,
                              const cudaSurfaceObject_t labelling,
                              const cudaSurfaceObject_t geom_buffer,//the depth and geometry at the current view
                              cv::Size2i resolution,
                              Matrix4f _pose,
                              Matrix4f proj_pose,
                              GpuVertex *vertices,
                              Vector2f *tex_pos,
                              GpuTriangle* triangles,
                              GpuPatchInfo* patch_infos) {

	if(tasks.empty()) {
		return;
	}
	dim3 block(256);
	dim3 grid(tasks.size());
	size_t task_bytes = sizeof(Labelling::SegProjTask) * tasks.size();
	Labelling::SegProjTask *tasks_gpu;
	cudaMalloc(&tasks_gpu, task_bytes);
	cudaMemcpy(tasks_gpu, &(tasks[0]), task_bytes, cudaMemcpyHostToDevice);

	//TODO: call the kernel
	labelSurfaces_kernel<<<grid, block>>>(tasks_gpu, labelling, geom_buffer,
	                                      resolution, _pose, proj_pose, vertices,
	                                      tex_pos, triangles, patch_infos);

	cudaFree(tasks_gpu);
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

template<class T> __global__ 
void initializeSurfaceLabels_kernel(Labelling::InitializeTask *tasks, T value) {
	uint32_t k = blockIdx.x;
	Labelling::InitializeTask &task = tasks[k];
	uint32_t i = threadIdx.x;

	int pixel_count = task.dest_rect.width * task.dest_rect.height;
	while(i < pixel_count) {

		int x = i % task.dest_rect.width;
		int y = i / task.dest_rect.width;

		int x_dest = x + task.dest_rect.x;
		int y_dest = y + task.dest_rect.y;

		surf2Dwrite(value, task.dest_surf, x_dest * sizeof(T), y_dest);

		i += blockDim.x;
	}
}

template<class T>
void Labelling::initializeSurfaces(vector<Labelling::InitializeTask> tasks, 
                                   T value) {
	if(tasks.empty()) {
		return;
	}
	dim3 block(256);
	dim3 grid(tasks.size());

	size_t task_bytes = sizeof(Labelling::InitializeTask) * tasks.size();
	Labelling::InitializeTask *tasks_gpu;
	cudaMalloc(&tasks_gpu, task_bytes);
	cudaMemcpy(tasks_gpu, &(tasks[0]), task_bytes, cudaMemcpyHostToDevice);

	initializeSurfaceLabels_kernel<<<grid, block>>>(tasks_gpu, value);

	cudaFree(tasks_gpu);
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

//specialization
template
void Labelling::initializeSurfaces(vector<Labelling::InitializeTask> tasks,
                                   int32_t value);

template<>
void Labelling::initializeSurfaces(vector<Labelling::InitializeTask> tasks,
                                   Vector4f value) {
	float4 value2 = make_float4(value[0], value[1], value[2], value[3]);
	Labelling::initializeSurfaces(tasks, value2);
}

} // namespace gpu
