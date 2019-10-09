#include "xtion_camera_model.h"

#include <assert.h>

#include "gpu_errchk.h"

__global__ 
void generateXtionConfidenceImage_kernel(const cudaSurfaceObject_t raw_depth,
                                         cudaSurfaceObject_t output,
                                         int width, int height) {

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	//exit the thread if we are out of bounds.
	if(x >= width || y >= height) {
		return;
	}

	float min_dist = 0.4;
	float a0 = 0.005; //5mm resolution for the closest distances
	float a1 = 0.0;
	float a2 = 0.01;

	float c = 4;

	//now calculate and store what we want.
	float depth;
	surf2Dread(&depth, raw_depth, x * sizeof(float), y);
	if(depth == 0) { //the depth never is zero, zero means invalid....
		depth = NAN;//NAN means invalid!!!!!
		//this automatically makes all the other calcultaions NAN as well.
	}
#ifdef OVERSIMPLIFY_CAM_MODEL
	float4 fixed = make_float4(depth, 0.1, 0.05, 1);
	surf2Dwrite(fixed, output, x * sizeof(float4), y);
	return;
#endif
	float delta = depth - min_dist;
	float sigma_m = a0 + delta * a1 + delta * delta * a2;//minimum standard deviation
	float sigma = c * sigma_m;

	//there is some kind of depth depending vignetting on the kinects
	float dignetting = 1.0f;
	float dx = float(x - 320) / 300.0f;
	float dy = float(y - 240) / 300.0f;
	dignetting = 1.0f + pow(dx * dx + dy * dy, 5);

	sigma_m = dignetting * sigma_m;
	sigma = dignetting * sigma;

	//of course we could also look at the neighbouring pixel to make a decicion for the standard deviation
	//and such

	float4 color = make_float4(depth, sigma, sigma_m, dignetting);
	surf2Dwrite(color, output, x * sizeof(float4), y);
}

void generateXtionConfidenceImage(const cudaSurfaceObject_t raw_depth,
                                  cudaSurfaceObject_t output,
                                  int width, int height) {
	gpuErrchk(cudaPeekAtLastError());
	dim3 block(32, 32);
	dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);

	generateXtionConfidenceImage_kernel<<<grid, block>>>(raw_depth, output, width, 
	                                                     height);
	cudaDeviceSynchronize(); //maybe we should work with streams, but what do i know?
	gpuErrchk(cudaPeekAtLastError());
}

__global__ 
void generateXtionConfidenceImage16F_kernel(const cudaSurfaceObject_t raw_depth,
                                            cudaSurfaceObject_t output,
                                            int width, int height) {

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	//exit the thread if we are out of bounds.
	if(x >= width || y >= height) {
		return;
	}

	float min_dist = 0.4;
	float a0 = 0.005; //5mm resolution for the closest distances
	float a1 = 0.0;
	float a2 = 0.01;

	float c = 4;
	//now calculate and store what we want.
	float depth;
	surf2Dread(&depth, raw_depth, x * sizeof(float), y);

	if(depth == 0) { //the depth never is zero, zero means invalid....
		depth = NAN;//NAN means invalid!!!!!
		//this automatically makes all the other calcultaions NAN as well.
	}
	float delta = depth - min_dist;
	float sigma_m = a0 + delta * a1 + delta * delta * a2;//minimum standard deviation
	float sigma = c * sigma_m;
	//of course we could also look at the neighbouring pixel to make a decicion for the standard deviation
	//and such

	float4 color = make_float4(depth, sigma,sigma_m, 1);
	ushort4 result = make_ushort4(__float2half_rn(color.x),
	                              __float2half_rn(color.y),
	                              __float2half_rn(color.z),
	                              __float2half_rn(color.w));
	surf2Dwrite(result, output, x * sizeof(ushort4), y);
}

void generateXtionConfidenceImage16F(const cudaSurfaceObject_t raw_depth,
                                     cudaSurfaceObject_t output,
                                     int width, int height) {
	assert(0);
	dim3 block(32, 32);
	dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);

	generateXtionConfidenceImage16F_kernel<<<grid, block>>>(raw_depth, output, 
	                                                        width, height);
	cudaDeviceSynchronize(); //maybe we should work with streams, but what do i know?
	gpuErrchk(cudaPeekAtLastError());
}
