#include <cuda/float16_utils.h>

#include <iostream>

#include <cuda/gpu_errchk.h>

using namespace std;

void castF16SurfaceToF32Buffer(cudaSurfaceObject_t surface, int x, int y,
                               int width, int height, float *buffer,
                               int channels) {
	F16SurfBufCastDesc task;
	task.x       = x;
	task.y       = y;
	task.width   = width;
	task.height  = height;
	task.surface = surface;
	task.offset  = 0;

	vector<F16SurfBufCastDesc> tasks;
	tasks.push_back(task);
	vector<uint8_t*> data;
	data.push_back((uint8_t*) buffer);
	castF16SurfaceToF32Buffers(tasks, data, channels);
}

template<int Channels> __global__
void castF16SurfaceToF32Buffers_kernel(F16SurfBufCastDesc *tasks,
                                       float *targets) {
	static_assert(Channels == 1 ||  Channels == 2 ||  Channels == 4,
	              "the texture has either to be 1, 2 or 4 channels\n");

	const int k = blockIdx.x;
	F16SurfBufCastDesc desc = tasks[k];

	int max_pix_count = desc.width * desc.height;
	int i = threadIdx.x;
	while(i < max_pix_count) {
		int x = i % desc.width + desc.x;
		int y = i / desc.width + desc.y;

		__half f16Var[4];
		switch(Channels) {
			case 1:
				surf2Dread((ushort*) f16Var, desc.surface, 
				           x * sizeof(uint16_t) * Channels, y);
				break;
			case 2:
				surf2Dread((ushort2*) f16Var, desc.surface, 
				           x * sizeof(uint16_t) * Channels, y);
				break;
			case 3:
				//a three color channel texture doesn't really exist
				break;
			case 4:
				surf2Dread((ushort4*) f16Var, desc.surface, 
				           x * sizeof(uint16_t) * Channels, y);
				break;
		}
		//convert and store
		for(size_t j = 0; j < Channels; j++) {
			targets[(desc.offset + i) * Channels + j] = __half2float(f16Var[j]);
		}
		i += blockDim.x;
	}
}

void castF16SurfaceToF32Buffers(vector<F16SurfBufCastDesc> tasks,
                                vector<uint8_t*> data, int channels) {
	//cout << "this is untested" << endl;

	F16SurfBufCastDesc *gpu_tasks;
	cudaMalloc(&gpu_tasks, sizeof(F16SurfBufCastDesc) * tasks.size());
	int pixel_count = 0;
	for(size_t i = 0; i < tasks.size(); i++) {
		tasks[i].offset = pixel_count;
		pixel_count += tasks[i].width * tasks[i].height;
	}
	float *targets;
	cudaMalloc(&targets, sizeof(float) * pixel_count * channels);
	cudaMemcpy(gpu_tasks, &tasks[0], sizeof(F16SurfBufCastDesc) * tasks.size(),
	           cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	dim3 block(1024);
	dim3 grid(tasks.size());

	switch(channels) {
		case 1:
			castF16SurfaceToF32Buffers_kernel<1><<<grid, block>>>(gpu_tasks, targets);
			break;
		case 2:
			castF16SurfaceToF32Buffers_kernel<2><<<grid, block>>>(gpu_tasks, targets);
			break;
		case 3:
			cout << "[castF16SurfaceToF32Buffers] "
			        "Three channels are not supported" << endl;
			assert(0);
			break;
		case 4:
			castF16SurfaceToF32Buffers_kernel<4><<<grid, block>>>(gpu_tasks, targets);
			break;
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	// Now download the data
	for(size_t i = 0; i < tasks.size(); i++) {
		// BUT THIS DIDN'T FAIL BEFORE!!!!!
		cudaMemcpy(data[i], &(targets[channels * tasks[i].offset]),
		           channels * sizeof(float) * tasks[i].width * tasks[i].height,
		           cudaMemcpyDeviceToHost);
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(targets);
	cudaFree(gpu_tasks);
}

void castF32BufferToF16Surface(cudaSurfaceObject_t surface, int x, int y,
                               int width, int height, float *buffer, 
                               int channels) {

	F16SurfBufCastDesc task;
	task.x       = x;
	task.y       = y;
	task.width   = width;
	task.height  = height;
	task.surface = surface;
	task.offset  = 0;

	vector<F16SurfBufCastDesc> tasks;
	tasks.push_back(task);
	vector<uint8_t*> data;
	data.push_back((uint8_t*) buffer);
	castF32BufferToF16Surfaces(tasks, data, channels);
}

template<int Channels> __global__
void castF32BufferToF16Surface_kernel(F16SurfBufCastDesc *tasks,
                                      float *sources) {
	const int k = blockIdx.x;
	F16SurfBufCastDesc desc = tasks[k];

	int max_pix_count = desc.width * desc.height;
	int i = threadIdx.x;
	while(i < max_pix_count) {
		int x = i % desc.width + desc.x;
		int y = i / desc.width + desc.y;

		float var[4];
		for(size_t j = 0; j < Channels; j++) {
			var[j] = sources[(desc.offset + i) * Channels + j];
		}
		//convert and store
		switch(Channels) {
			case 1:
				surf2Dwrite((ushort) __float2half_rn(var[0]), desc.surface, 
                    x * sizeof(uint16_t) * Channels, y);
				break;
			case 2:
				surf2Dwrite(
						make_ushort2(__float2half_rn(var[0]), __float2half_rn(var[1])), 
						desc.surface, x * sizeof(uint16_t) * Channels, y);
				break;
			case 3:
				//a three color channel texture doesn't really exist
				break;
			case 4:
				surf2Dwrite(make_ushort4(__float2half_rn(var[0]),
				                         __float2half_rn(var[1]),
				                         __float2half_rn(var[2]),
				                         __float2half_rn(var[3])),
				            desc.surface, x * sizeof(uint16_t) * Channels, y);
				break;
		}
		i += blockDim.x;
	}
}

void castF32BufferToF16Surfaces(vector<F16SurfBufCastDesc> tasks,
                                vector<uint8_t*> data, int channels) {
	if(tasks.empty()) {
		return;
	}

	F16SurfBufCastDesc* gpu_tasks;
	cudaMalloc(&gpu_tasks, sizeof(F16SurfBufCastDesc) * tasks.size());
	int pixel_count=0;
	for(size_t i = 0; i < tasks.size(); i++) {
		tasks[i].offset = pixel_count;
		pixel_count += tasks[i].width * tasks[i].height;
	}
	float *targets;
	cudaMalloc(&targets, sizeof(float) * pixel_count * channels);
	cudaMemcpy(gpu_tasks, &tasks[0], sizeof(F16SurfBufCastDesc) * tasks.size(),
	           cudaMemcpyHostToDevice);

	//also upload the images from the cpu to the gpu buffer
	for(size_t i = 0; i < tasks.size(); i++) {
		cudaMemcpy(&(targets[channels * tasks[i].offset]), data[i],
		           channels * sizeof(float) * tasks[i].width * tasks[i].height,
		           cudaMemcpyHostToDevice);
	}

	dim3 block(1024);
	dim3 grid(tasks.size());

	switch(channels) {
		case 1:
			castF32BufferToF16Surface_kernel<1><<<grid, block>>>(gpu_tasks, targets);
			break;
		case 2:
			castF32BufferToF16Surface_kernel<2><<<grid, block>>>(gpu_tasks, targets);
			break;
		case 3:
			cout << "three channels are not supported" << endl;
			assert(0);
			break;
		case 4:
			castF32BufferToF16Surface_kernel<4><<<grid, block>>>(gpu_tasks, targets);
			break;
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	cudaFree(targets);
	cudaFree(gpu_tasks);
}