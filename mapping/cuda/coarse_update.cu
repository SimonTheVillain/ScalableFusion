#include "coarse_update.h"

//stupid debug shit!!!
#include <iostream>

#include "gpu_errchk.h"

using namespace std;
using namespace Eigen;

__global__ 
void reduceColorAtPatches_kernel(CalcMeanColorDescriptor *descriptors,
                                 GpuCoarseVertex *vertices, Vector4f *colors) {
	extern __shared__ Vector4f s_colors[];//this should not necessarily hardcoded
	const int k = blockIdx.x;
	Vector4f color(0, 0, 0, 0);

	//sum up over all the elements
	CalcMeanColorDescriptor &desc = descriptors[k];

	if(desc.vert_ind < 0) {
		return;
	}

	int absolute_pix_count = desc.height * desc.width;

	int i = threadIdx.x;
	while(i < absolute_pix_count) {
		int x = i % desc.width + desc.x;
		int y = i / desc.width + desc.y;

		Vector4f pix_color;
		if(desc.hdr) {
			//if the image is stored as hdr we load it as such
			printf("TODO!!!!! \n");
		} else {
			//otherwise we load it as 8 bit per color image
			uchar4 c;
			surf2Dread(&c, desc.color, x * sizeof(uchar4), y);
			pix_color = Vector4f(c.x, c.y, c.z, c.w) * (desc.scale / 255.0f);
		}
		color += pix_color;

		i += blockDim.x;
	}

	//Do reduction to get the mean color
	int tid = threadIdx.x;
	s_colors[tid] = color;
	__syncthreads();

	for(int s = blockDim.x / 2; s > 0; s >>= 1) {
		if(tid >= s) {
			//return;//TODO: maybe add this back in!?
		}
		if(tid < s) {
			s_colors[tid] += s_colors[tid + s];
		}
		__syncthreads();
	}
	if(tid == 0 &&  desc.vert_ind >= 0) {
		vertices[desc.vert_ind].c = s_colors[0] * (1.0f / float(absolute_pix_count));
	}
	//we should be able to get rid of the loop
	//it will be different beginning with cuda 9 though
	//http://developer.download.nvidia.com/compute/cuda/1.1-Beta/x86_website/projects/reduction/doc/reduction.pdf
}

void calcMeanColor(vector<CalcMeanColorDescriptor> descriptors,
                   GpuCoarseVertex* vertices, Vector4f* colors) {
	if(descriptors.empty()) {
		return;
	}
		//lets do 1024 threads for each patch
	dim3 block(1024);
	dim3 grid(descriptors.size());

	Vector4f* colors_gpu = 0;
	if(colors) {
		cudaMalloc(&colors_gpu, descriptors.size() * sizeof(Vector4f));
	}

	CalcMeanColorDescriptor *descs;
	cudaMalloc(&descs, descriptors.size() * sizeof(CalcMeanColorDescriptor));
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(CalcMeanColorDescriptor), 
	           cudaMemcpyHostToDevice);

	gpuErrchk(cudaPeekAtLastError());

	reduceColorAtPatches_kernel<<<grid, block, sizeof(Vector4f) * block.x>>>(
			descs,vertices,colors_gpu);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	cudaFree(descs);

	if(colors) {
		cudaMemcpy(colors_gpu, colors, descriptors.size() * sizeof(Vector4f), 
		           cudaMemcpyDeviceToHost);
		cudaFree(colors_gpu);
		cudaDeviceSynchronize();
		gpuErrchk(cudaPeekAtLastError());
	}
}

__global__ 
void setVisibility_kernel(int *visib_indices, size_t nr_indices,
                          int *gpu_vis_buffer, int value) {
	const int i = threadIdx.x + blockIdx.x * blockDim.x;
	if(i >= nr_indices) {
		return;
	}
	int index = visib_indices[i];
	gpu_vis_buffer[index] = value;
}

void coarseUpdateVisibility(vector<int> enable_patches, 
                            vector<int> disable_patches,
                            int *gpu_vis_buffer) {
	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	int *data;
	cudaMalloc(&data, 
	           sizeof(int) * max(enable_patches.size(), disable_patches.size()));

	dim3 block(256);
	dim3 grid(enable_patches.size() / block.x + 1);

	cudaMemcpy(data, &enable_patches[0], enable_patches.size() * sizeof(int),
	           cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	if(!enable_patches.empty()) {
		setVisibility_kernel<<<grid, block>>>(data, enable_patches.size(), 
		                                      gpu_vis_buffer, 1);
	}

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	grid.x = disable_patches.size() / block.x + 1;

	cudaMemcpy(data, &disable_patches[0], disable_patches.size() * sizeof(int),
	          cudaMemcpyHostToDevice);
	if(!disable_patches.empty()) {
		setVisibility_kernel<<<grid, block>>>(data, disable_patches.size(),
		                                      gpu_vis_buffer, 0);
	}

	cudaError err = cudaPeekAtLastError();
	if(err != cudaSuccess) {
		cout << data << endl;// i fear the pointer is empty for some weird reason
	}
	cudaDeviceSynchronize();
	gpuErrchk(err);

	cudaFree(data);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}