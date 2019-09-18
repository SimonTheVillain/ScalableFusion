#include "reproject.h"

#include <stdio.h>

using namespace Eigen;

__global__
void reproject_kernel(uint16_t *depth_in, uint32_t *depth_out,
                      int width, int height,
                      Matrix4f pose_transform,
                      Vector4f depth_intrinsics) {

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if(x >= width || y >= height) {
		return;
	}

	//depth
	float fx_d = depth_intrinsics[0];
	float fy_d = depth_intrinsics[1];
	float cx_d = depth_intrinsics[2];
	float cy_d = depth_intrinsics[3];
	float z = float(depth_in[x + y * width]) * 0.001f; // from millimeter to meter

	if(z == 0) {
		return;
	}

	Vector4f p((float(x) - cx_d) * z / fx_d,
	           (float(y) - cy_d) * z / fy_d,
	           z,
	           1.0f);

	Vector4f pnew= pose_transform * p;

	// Now project:
	float z_target = pnew[2];
	float x_target = pnew[0] / z_target * fx_d + cx_d;
	float y_target = pnew[1] / z_target * fy_d + cy_d;

	int xt = round(x_target);
	int yt = round(y_target);

	if(xt >= width || yt >= height || xt < 0 || yt < 0) {
		return;
	}

	atomicMin(&depth_out[xt + yt * width], uint32_t(z_target * 1000.0f));
}

__global__
void typecastAndMasking_kernel(uint16_t *depth_out16, uint32_t *depth_out32,
                               int width, int height) {

	int x = blockIdx.x * blockDim.x + threadIdx.x;
	int y = blockIdx.y * blockDim.y + threadIdx.y;

	if(x >= width || y >= height) {
		return;
	}

	uint32_t z = depth_out32[x +  y * width];
	if(z > 65000) {
		depth_out16[x +  y * width] = 0;
	} else {
		depth_out16[x +  y * width] = z;
	}
}

void reproject(uint16_t *depth_in, uint16_t *depth_out16, uint32_t *depth_out32,
               int width, int height, Matrix4f pose_transform, 
               Vector4f depth_intrinsics) {

	dim3 block(32,32);
	dim3 grid((width + block.x - 1) / block.x, (height + block.y - 1) / block.y);

	reproject_kernel<<<grid, block>>>(depth_in, depth_out32, width, height,
	                                  pose_transform, depth_intrinsics);

	typecastAndMasking_kernel<<<grid, block>>>(depth_out16, depth_out32, width,
	                                           height);

}
