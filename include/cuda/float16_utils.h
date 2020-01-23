#ifndef FILE_FLOAT16_UTILS_H
#define FILE_FLOAT16_UTILS_H

#include <vector>

#include <cuda.h>
#include <cublas.h>

/**
	* We want to download the float 16 values to the cpu and directly interpret them,
	* therefore we need to convert them to float32 on gpu.... uploading the data yields
	* a similar issue. ( i know this could be a performance issue)
	*/

using namespace std;

struct F16SurfBufCastDesc {
	cudaSurfaceObject_t surface;
	int x;
	int y;
	int width;
	int height;
	int offset; //filled out by the function we want to use
};

void castF16SurfaceToF32Buffer(cudaSurfaceObject_t surface, int x, int y,
                               int width, int height, float *buffer,
                               int channels);

void castF16SurfaceToF32Buffers(vector<F16SurfBufCastDesc> tasks, 
                                vector<uint8_t*> data, int channels);

void castF32CPUBufferToF16Surface(cudaSurfaceObject_t surface, int x, int y,
                               int width, int height, float *buffer, 
                               int channels);

void castF32CPUBufferToF16Surfaces(vector<F16SurfBufCastDesc> tasks,
                                vector<uint8_t*> data, int channels);

#ifdef __CUDACC__
inline __host__ __device__ 
ushort4 float4_2_half4_reinterpret_ushort4_rn(float4 in) {
	__half2 h2 = __floats2half2_rn(in.x, in.y);
		union {
			__half2 f[2];
			ushort4 i;
		} u;
		u.f[0] = __floats2half2_rn(in.x, in.y);
		u.f[1] = __floats2half2_rn(in.z, in.w);
		return u.i;
}
#endif

#endif // FILE_FLOAT16_UTILS_H