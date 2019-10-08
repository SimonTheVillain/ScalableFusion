#include "coalescedMemoryTransfer.h"

#include <iostream>

#include "gpuErrchk.h"

using namespace std;
using namespace Eigen;

__global__
void downloadVertices_kernel(GpuVertex **ptrs, GpuVertex* out, int count) {
	const int i = threadIdx.x + blockIdx.x * blockDim.x;
	if(i >= count) {
		return;
	}

	out[i]= *(ptrs[i]); // Looking at this it would be a pain not to template this:

	//TWO methods:
	//gather: collect spread stuff in an array
	//scatter: fill spread containers with data
}

void downloadVertices(vector<GpuVertex*> gpu_vertices, GpuVertex *data) {
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	const int size = gpu_vertices.size();
	GpuVertex* gpu_data;
	cudaMalloc(&gpu_data, sizeof(GpuVertex) * size);
	GpuVertex** gpu_ptrs;
	cudaMalloc(&gpu_ptrs, sizeof(GpuVertex*) * size);
	cudaMemcpy(gpu_ptrs,&(gpu_vertices[0]), sizeof(GpuVertex*) * size,
	           cudaMemcpyHostToDevice);

	dim3 block(512);
	dim3 grid(size / block.x + 1);
	downloadVertices_kernel<<<grid, block>>>(gpu_ptrs, gpu_data, size);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaMemcpy(&(data[0]), gpu_data, sizeof(GpuVertex) *size,
	           cudaMemcpyDeviceToHost);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(gpu_data);
	cudaFree(gpu_ptrs);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

template<typename T> __global__ 
void upload_kernel(T *source, CoalescedGpuTransfer::Task *tasks) {
	CoalescedGpuTransfer::Task task = tasks[blockIdx.x];
	int i = threadIdx.x;
	while(i < task.count) {
		T unit = source[task.start + i];
		((T*) (task.target))[i] = unit;
		i += blockDim.x;
	}
}

template<typename T>
void CoalescedGpuTransfer::upload(vector<T> source,
                                  vector<CoalescedGpuTransfer::Task> tasks) {
	if(tasks.empty()) {
		return;
	}
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	dim3 block(256);
	dim3 grid(tasks.size());

	T *gpu_src;
	CoalescedGpuTransfer::Task *gpu_tasks;
	cudaMalloc(&gpu_src, sizeof(T) * source.size());
	cudaMalloc(&gpu_tasks, sizeof(CoalescedGpuTransfer::Task) * tasks.size());
	cudaMemcpy(gpu_src, &(source[0]), sizeof(T)*source.size(), 
	           cudaMemcpyHostToDevice);
	cudaMemcpy(gpu_tasks, &(tasks[0]), 
	           sizeof(CoalescedGpuTransfer::Task) * tasks.size(),
	           cudaMemcpyHostToDevice);

	upload_kernel<<<grid, block>>>(gpu_src, gpu_tasks);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(gpu_src);
	cudaFree(gpu_tasks);
}

template<typename T> __global__ 
void upload_kernel(T *src, T **dst, int size) {
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	if(i < size) {
		*(dst[i]) = src[i];
	}
}

template<typename T>
void CoalescedGpuTransfer::upload(vector<T> source, vector<T*> gpu_dst) {
	if(source.size() != gpu_dst.size()) {
		assert(0); //you are misusing this and you deserve your program to crash
	}
	if(source.empty()) {
		return;
	}
	dim3 block(256);
	dim3 grid(source.size() / block.x + 1);
	T *src;
	T **dst;

	cudaMalloc(&src, sizeof(T) * source.size());
	cudaMalloc(&dst, sizeof(T*) * source.size());

	cudaMemcpy(src, &(source[0]), sizeof(T) * source.size(),
	           cudaMemcpyHostToDevice);
	cudaMemcpy(dst, &(gpu_dst[0]), sizeof(T*) * source.size(),
	           cudaMemcpyHostToDevice);

	upload_kernel<<<grid, block>>>(src, dst, source.size());

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(src);
	cudaFree(dst);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

template<typename T> __global__ 
void device2DeviceSameBuf_kernel(T *buf, CoalescedGpuTransfer::TaskD2D *tasks) {
	CoalescedGpuTransfer::TaskD2D task = tasks[blockIdx.x];
	int i = threadIdx.x;
	while(i < task.count) {
		buf[i + task.destination_index] = buf[i + task.source_index];
		i += blockDim.x;
	}
}

template<typename T>
void CoalescedGpuTransfer::device2DeviceSameBuf(T *buffer, 
                                                vector<TaskD2D> tasks) {

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(tasks.empty()) {
		return;
	}

	size_t bytes = sizeof(TaskD2D) * tasks.size();
	TaskD2D* tasks_gpu;
	cudaMalloc(&tasks_gpu, bytes);
	cudaMemcpy(static_cast<void*>(tasks_gpu), static_cast<void*>(&(tasks[0])),
	           bytes, cudaMemcpyHostToDevice);

	dim3 block(256);
	dim3 grid(tasks.size());

	device2DeviceSameBuf_kernel<<<grid, block>>>(buffer, tasks_gpu);

	cudaFree(tasks_gpu);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

//THIS IS NOT EFFICIENT
template<typename T> __global__ 
void upload_kernel(CoalescedGpuTransfer::SetTaskTemp<T> *tasks, int count) {
	const int k = blockIdx.x * blockDim.x + threadIdx.x;
	if(k >= count) {
		return;
	}
	*(tasks[k].dst) = tasks[k].value;
}

template<typename T>
void CoalescedGpuTransfer::upload(vector<SetTaskTemp<T>> tasks) {
	cout << "THIS IS NOT EFFICIENT!!!" << endl;
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(tasks.size() == 0) {
		return;
	}
	SetTaskTemp<T> *tasks_gpu;
	size_t bytes = sizeof(SetTaskTemp<T>) * tasks.size();

	cudaMalloc(&tasks_gpu, bytes);
	cudaMemcpy(static_cast<void*>(tasks_gpu), static_cast<void*>(&(tasks[0])),
	           bytes, cudaMemcpyHostToDevice);

	dim3 block(256);
	dim3 grid(tasks.size() / block.x + 1);

	upload_kernel<<<grid, block>>>(tasks_gpu, tasks.size());

	//this is going to be disgusting:
	for(size_t i = 0; i < tasks.size(); i++) {
		//doing these copies one after another (inefficiently like i dont care)
		cudaMemcpy(static_cast<void*>(tasks[i].dst), 
		           static_cast<void*>(&(tasks[i].value)),
		           sizeof(SetTaskTemp<T>),
		           cudaMemcpyHostToDevice);
	}

	cudaFree(tasks_gpu);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

//THIS IS NOT EFFICIENT
template<typename T> __global__ 
void copy_kernel(CoalescedGpuTransfer::CpyTaskTemp<T>* tasks, int count) {
	const int k = blockIdx.x * blockDim.x + threadIdx.x;
	if(k >= count) {
		return;
	}
	*(tasks[k].dst) = *(tasks[k].src);
}

template<typename T>
void CoalescedGpuTransfer::copy(vector<CpyTaskTemp<T>> tasks) {
	cout << "THIS IS NOT EFFICIENT!!!" << endl;
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(tasks.size() == 0) {
		return;
	}
	CpyTaskTemp<T> *tasks_gpu;
	size_t bytes = sizeof(CpyTaskTemp<T>) * tasks.size();

	cudaMalloc(&tasks_gpu, bytes);
	cudaMemcpy(static_cast<void*>(tasks_gpu), static_cast<void*>(&(tasks[0])),
	           bytes, cudaMemcpyHostToDevice);

	dim3 block(256);
	dim3 grid(tasks.size() / block.x +1);

	copy_kernel<<<grid, block>>>(tasks_gpu, tasks.size());

	cudaFree(tasks_gpu);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

__global__ 
void download_kernel(CoalescedGpuTransfer::DirectTask *tasks) {
	const int k = blockIdx.x;
	CoalescedGpuTransfer::DirectTask task = tasks[k];
	int i = threadIdx.x;
	uint32_t *src = static_cast<uint32_t*>(task.src);
	uint32_t *dst = static_cast<uint32_t*>(task.dst);

	int words = task.byteCount / 4;
	while(i < words) {
		dst[i] = src[i];
		i += blockDim.x;
	}

	i = threadIdx.x;
	int leftover = task.byteCount % 4;
	if(i < leftover) {
		uint8_t *src8 = static_cast<uint8_t*>(task.src);
		uint8_t *dst8 = static_cast<uint8_t*>(task.dst);
		dst8[task.byteCount - leftover + i] = src8[task.byteCount - leftover + i];
	}
}

void CoalescedGpuTransfer::download(
		vector<CoalescedGpuTransfer::DirectTask> tasks) {
	if(tasks.empty()) {
		return;
	}

	int word_count = 0; //count of 4 byte values

	//list of starting indices
	vector<int> starting_indices(tasks.size());
	for(int i = 0; i < tasks.size(); i++) {
		starting_indices[i] = word_count;
		word_count += tasks[i].byteCount / 4;
		if(tasks[i].byteCount % 4) {
			word_count++;
		}
	}

	//setup continuous buffer to store
	int *result_gpu;

	cudaMalloc(&result_gpu, word_count * 4);

	vector<CoalescedGpuTransfer::DirectTask> tasks2 = tasks;
	for(int i = 0; i < tasks.size(); i++) {
		//instead of the destination pointing to cpu memory as in tasks in tasks2 it should point to GPU mem.
		tasks2[i].dst = &result_gpu[starting_indices[i]];
	}

	//setup and fill buffer for task list
	CoalescedGpuTransfer::DirectTask *tasks_gpu;
	size_t bytes = sizeof(CoalescedGpuTransfer::DirectTask) * tasks.size();
	cudaMalloc(&tasks_gpu, bytes);
	cudaMemcpy(static_cast<void*>(tasks_gpu), static_cast<void*>(&(tasks2[0])),
	           bytes, cudaMemcpyHostToDevice);

	//run kernel
	dim3 block(256);
	//dim3 grid(tasks.size()*(sizeof(TaskTemp<T>)/16 + 2);//16 byte alignment
	dim3 grid(tasks.size());
	download_kernel<<<grid, block>>>(tasks_gpu);

	//download continuous buffer
	vector<uint32_t> result(word_count * 4);
	cudaMemcpy(static_cast<void*>(&(result[0])), static_cast<void*>(result_gpu),
	           word_count * 4, cudaMemcpyDeviceToHost);

	//destroy buffers
	cudaFree(result_gpu);
	cudaFree(tasks_gpu);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//fill everything to target
	for(int i = 0; i < tasks.size(); i++) {
		//copy the results to the real target...
		memcpy(tasks[i].dst, &result[starting_indices[i]], tasks[i].byteCount);
	}
}

//instantiate this templated method
template void CoalescedGpuTransfer::upload(vector<GpuPatchInfo> source,
								vector<GpuPatchInfo*> gpuDst);

//instantiate these templated methods
template void CoalescedGpuTransfer::upload(vector<GpuVertex> source,
								vector<CoalescedGpuTransfer::Task> tasks);
template void CoalescedGpuTransfer::upload(vector<GpuTriangle> source,
								vector<CoalescedGpuTransfer::Task> tasks);

template void CoalescedGpuTransfer::upload(vector<Eigen::Vector2f> source,
								vector<CoalescedGpuTransfer::Task> tasks);

template void CoalescedGpuTransfer::device2DeviceSameBuf(Eigen::Vector2f* buf,
vector<CoalescedGpuTransfer::TaskD2D> tasks);

template void CoalescedGpuTransfer::upload(vector<CoalescedGpuTransfer::SetTaskTemp<GpuTextureInfo>> tasks);

template void CoalescedGpuTransfer::copy(vector<CoalescedGpuTransfer::CpyTaskTemp<GpuTextureInfo>> tasks);
