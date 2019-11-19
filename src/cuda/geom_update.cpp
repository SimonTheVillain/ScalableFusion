#include <cuda/geom_update.h>

#include <base/mesh_structure.h>

using namespace std;
using namespace Eigen;

void gpu::GeometryUpdate::calcCenterAndRadius(
		vector<shared_ptr<MeshPatch>> &patches) {
/*
	if(patches.empty()) {
		return;
	}
	vector<gpu::GeometryUpdate::CalcCenterTask> tasks;

	int debug = 0;
	for(shared_ptr<MeshPatch> patch : patches) {
		gpu::GeometryUpdate::CalcCenterTask task;
		shared_ptr<MeshPatchGpuHandle> gpu_patch = patch->gpu.lock();
		if(gpu_patch == nullptr) {
			assert(0);
		}
		shared_ptr<VertexBufConnector> buffer = gpu_patch->vertices_source;
		task.vertices = buffer->getStartingPtr();
		task.count = buffer->getSize();
		tasks.push_back(task);
		debug++;
	}

	//allocate gpu memory
	gpu::GeometryUpdate::CalcCenterTask *gpu_tasks;
	size_t bytes = sizeof(gpu::GeometryUpdate::CalcCenterTask) * tasks.size();
	cudaMalloc(&gpu_tasks, bytes);
	cudaMemcpy(gpu_tasks, &tasks[0], bytes, cudaMemcpyHostToDevice);
	Vector4f *gpu_centers;
	cudaMalloc(&gpu_centers, sizeof(Vector4f) * tasks.size());

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//calc center and radius in one run
	dim3 block(256);
	dim3 grid(tasks.size());
	bytes = sizeof(Vector4f) * block.x;
	calcCenterAndRadiusKernelCall_(grid, block, bytes, gpu_tasks, gpu_centers);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	Vector4f centers[tasks.size()];
	cudaMemcpy(centers, gpu_centers, sizeof(Vector4f) * tasks.size(),
	           cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(gpu_tasks);
	cudaFree(gpu_centers);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	for(size_t i = 0; i < tasks.size(); i++) {
		//TODO: put this back in, because it is crashing

		patches[i]->setSphere(centers[i].block<3, 1>(0, 0), sqrt(centers[i][3]));

	}
	//calc radius

	//download it,

	//and last step:
	//maybe even set this stuff on the patch itself. or all patches at once?
	//it at least needs to be efficiently set for the octree
	*/
}