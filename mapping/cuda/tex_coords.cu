#include "tex_coords.h"

#include "gpu_errchk.h"

using namespace std;
using namespace Eigen;

__global__ 
void genTexCoords_kernel(TexCoordGen::Task *tasks, Matrix4f proj_pose,
                         GpuPatchInfo *patch_infos, GpuVertex* vertices) {
	int k = blockIdx.x;
	TexCoordGen::Task &task = tasks[k];
	int i = threadIdx.x;
	//ETC: but actually i should work on something different

	//do everything that also is in scaleableMapTexturing.cpp
	while(i < task.triangle_count) {
		for(size_t j = 0; j < 3; j++) {
			//get the vertex position of this triangle:
			GpuTriangle triangle = task.triangles[i];
			GpuPatchInfo patch_info= patch_infos[triangle.patch_info_inds[j]];
			int ind = patch_info.vertex_source_start_ind + triangle.indices[j];
			Vector4f p = vertices[ind].p;

			//now do the projection
			Vector4f proj = proj_pose * p;

			//and store the texture coordinate
			Vector2f tex_coord = Vector2f(proj[0] / proj[3], proj[1] / proj[3]);

			Vector2f scaled = Vector2f((tex_coord[0] - task.offset_x) * task.scale_x,
			                           (tex_coord[1] - task.offset_y) * task.scale_y);

			//TODO: put the point back into bounds
			task.coords[task.triangles[i].tex_indices[j]] = scaled;
		}
		i += blockDim.x;
	}
	//ooh shit this also needs the bounds
}


void TexCoordGen::genTexCoords(vector<TexCoordGen::Task> tasks, 
                               Matrix4f proj_pose, GpuPatchInfo *patch_infos,
                               GpuVertex *gpu_vertices) {
	if(tasks.empty()) {
		return;
	}
	TexCoordGen::Task *gpu_tasks;
	int bytes = sizeof(TexCoordGen::Task) * tasks.size();
	cudaMalloc(&gpu_tasks, sizeof(TexCoordGen::Task) * tasks.size());
	cudaMemcpy(gpu_tasks, &(tasks[0]), bytes, cudaMemcpyHostToDevice);

	dim3 block(128);
	dim3 grid(tasks.size());

	genTexCoords_kernel<<<grid, block>>>(gpu_tasks, proj_pose, patch_infos,
	                                     gpu_vertices);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(gpu_tasks);
}

struct BoundResult {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Vector4f bound;
	int32_t target_ind = -1;
	uint32_t placeholders[3];
};

__global__ 
void getTexCoordBounds_kernel(TexCoordGen::BoundTask *tasks,
                              Matrix4f proj_pose, GpuPatchInfo *patch_infos,
                              GpuVertex *vertices, BoundResult *results) {
	__shared__ extern Vector4f bounds[];

	int k = blockIdx.x;
	TexCoordGen::BoundTask &task = tasks[k];
	int i = threadIdx.x;

	//ETC: but actually i should work on something different
	Vector4f bound(FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX);

	//do everything that also is in scaleableMapTexturing.cpp
	while(i < task.triangle_count) {
		GpuTriangle triangle = task.triangles[i];
		for(size_t j = 0; j < 3; j++) {
			//get the vertex position of this triangle:
			GpuPatchInfo patch_info = patch_infos[triangle.patch_info_inds[j]];
			int ind = patch_info.vertex_source_start_ind + triangle.indices[j];
			Vector4f p = vertices[ind].p;

			//now do the projection
			Vector4f proj = proj_pose * p;

			//and store the texture coordinate
			Vector2f tex_coord = Vector2f(proj[0] / proj[3], proj[1] / proj[3]);
			bound[0] = min(bound[0], tex_coord[0]);
			bound[1] = min(bound[1], tex_coord[1]);
			bound[2] = max(bound[2], tex_coord[0]);
			bound[3] = max(bound[3], tex_coord[1]);

			if(tex_coord[0] < -6000) {
				//TODO: find out why this tex coord is zero and handle any issues coming with this
				printf("type = %d, p= %f %f %f %f \n uv= %f %f \n task index = %d, traingle %d\n [texCoords.cu]/whyever this fails\n",
					task.debug_type, p[0], p[1], p[2], p[3], tex_coord[0], tex_coord[1], k, triangle.indices[j]);
			}
		}
		i += blockDim.x;
	}

	int tid = threadIdx.x;
	bounds[tid] = bound;
	__syncthreads();
	for(int s = blockDim.x / 2; s > 0; s >>= 1) {
		if(tid >= s) {
			//return;
		}
		if(tid < s) {
			bounds[tid][0] = min(bounds[tid][0], bounds[tid + s][0]);
			bounds[tid][1] = min(bounds[tid][1], bounds[tid + s][1]);
			bounds[tid][2] = max(bounds[tid][2], bounds[tid + s][2]);
			bounds[tid][3] = max(bounds[tid][3], bounds[tid + s][3]);
		}
		__syncthreads();
	}
	if(tid == 0) {
		results[k].bound = bounds[0];
		results[k].target_ind = task.target_ind;
		//TODO: output
	}
}

vector<cv::Rect2f> TexCoordGen::getPotentialTexCoordBounds(
		vector<TexCoordGen::BoundTask> tasks, Matrix4f proj_pose, int result_count,
		GpuPatchInfo *patch_infos, GpuVertex *vertices) {
	if(tasks.empty()) {
		vector<cv::Rect2f> empty;
		return empty;
	}
	TexCoordGen::BoundTask *gpu_tasks;
	cudaMalloc(&gpu_tasks, sizeof(TexCoordGen::BoundTask) * tasks.size());
	BoundResult *gpu_results;
	cudaMalloc(&gpu_results, sizeof(BoundResult) * tasks.size());
	cudaMemcpy(gpu_tasks, &(tasks[0]), 
	           sizeof(TexCoordGen::BoundTask) * tasks.size(),
	           cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	dim3 block(128);
	dim3 grid(tasks.size());
	int shared = block.x * sizeof(Vector4f);
	if(grid.x > 65000) {
		grid.x = 65000;
		printf("This is too many elements in the grid! fix this");
	}
	getTexCoordBounds_kernel<<<grid, block, shared>>>(gpu_tasks, proj_pose,
	                                                  patch_infos, vertices, 
	                                                  gpu_results);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	vector<BoundResult> raw_res(tasks.size());
	cudaMemcpy(&(raw_res[0]), gpu_results, sizeof(BoundResult) * tasks.size(),
	           cudaMemcpyDeviceToHost);

	cudaFree(gpu_tasks);
	cudaFree(gpu_results);

	vector<Vector4f> results(result_count, 
	                         Vector4f(FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX));

	for(size_t i = 0; i < raw_res.size(); i++) {
		BoundResult result = raw_res[i];

		int ind = result.target_ind;

		results[ind][0] = min(results[ind][0], result.bound[0]);
		results[ind][1] = min(results[ind][1], result.bound[1]);
		results[ind][2] = max(results[ind][2], result.bound[2]);
		results[ind][3] = max(results[ind][3], result.bound[3]);
	}

	vector<cv::Rect2f> bounds(result_count);
	for(size_t i = 0; i < bounds.size(); i++) {
		bounds[i].x = results[i][0];
		bounds[i].y = results[i][1];
		bounds[i].width = results[i][2] - results[i][0];
		bounds[i].height = results[i][3] - results[i][1];
	}
	return bounds;
}