#include "test.h"

struct RGBStorage {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

__global__ 
void copy_kernel(const cudaTextureObject_t texture,  
                 cv::cuda::PtrStepSz<RGBStorage> to) {

	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;

	if(x >= to.cols || y >= to.rows) {
		return;//out of bounds
	}

	RGBStorage point;

	float4 buff = tex2D<float4>(texture, x, y);

	point.r = buff.x * 255.0;
	if(x == 0) {
		//printf("%d\n",y);
	}
	point.g = buff.y * 255.0;
	point.b = buff.z * 255.0;

	to(y, x) = point;
}

void copy(cudaTextureObject_t texture, cv::cuda::GpuMat &to) {
	dim3 block(32, 32);
	dim3 grid((to.cols + block.x - 1) / block.x, 
	          (to.rows + block.y - 1) / block.y);
	copy_kernel<<<grid, block>>>(texture, to);
	cudaDeviceSynchronize();
}
