#ifndef FILE_SURFACE_READ_H
#define FILE_SURFACE_READ_H

#include <assert.h>

#include "helper_math.h"

#ifdef __CUDACC__

__device__ 
inline float2 unnormalizeTexCoords(float2 in, cv::Rect2i &rect) {
	return make_float2(in.x * float(rect.width - 1) + rect.x,
	                   in.y * float(rect.height - 1) + rect.y);
}

//TODO: adapt this for texAtlas
__device__ 
inline float4 readSensor(float x, float y, const cudaSurfaceObject_t surface,
                         int width, int height, float threshold) {

	// with the new version lets just take the closest:
	int x_closest = x - 0.5f;
	int y_closest = y - 0.5f;

	if(x_closest < 0 || y_closest < 0 || 
	   x_closest >= width || y_closest >= height) {
		return make_float4(NAN, NAN, NAN, NAN);
	}
	float4 closest;
	surf2Dread(&closest, surface, int(x_closest) * sizeof(float4),
	           int(y_closest));

	//return closest;
	//This is the old version that interpolates unconditionally!
	//which is wrong of course!

	//there is a lot that plays into this
	//think about it simon
	if(x < 0 || y < 0 || x > (width - 1) || y > (height - 1)) {
		 if(x == 0 && blockIdx.x == 0) {
			 //printf("shit are we out of bounds? %f of %d \n",y,height);
		 }
		return make_float4(NAN, NAN, NAN, NAN);
	}

	float x_tmp = x - float(int(x));
	float y_tmp = y - float(int(y));

	if(x_tmp == 0 && int(x) == width - 1) {
		x_tmp = 1.0f;
		x = x - 1.0f;
	}

	if(y_tmp == 0 && int(y) == height-1) {
		y_tmp = 1.0f;
		y = y - 1.0f;
	}

	//bottom:
	float4 Q00;
	surf2Dread(&Q00, surface, int(x) * sizeof(float4), int(y));
	float4 Q01;
	surf2Dread(&Q01, surface, (int(x) + 1) * sizeof(float4), int(y));
	float4 Q0x = x_tmp * Q01 + (1.0f - x_tmp) * Q00;

	//top
	float4 Q10;
	surf2Dread(&Q10, surface, int(x) * sizeof(float4), int(y) + 1);
	float4 Q11;
	surf2Dread(&Q11, surface, int(x) * sizeof(float4), int(y) + 1);
	float4 Q1x = x_tmp * Q11 + (1.0f - x_tmp) * Q10;

	if(abs(Q00.x - Q10.x) > threshold ||
	   abs(Q01.x - Q11.x) > threshold ||
	   abs(Q00.x - Q01.x) > threshold) {
		//in case all the points are close in depth we interpolate
		// if not we just put out the pixel to this sample position
		return closest;
	}
	//mix the middle;
	float4 Qyx = y_tmp * Q1x + (1.0f - y_tmp) * Q0x;

	return Qyx;
}

/**
 * @brief readBilinear
 * This is a function to read interpolated pixel from a surface object,
 * The pixel center are at full integer positions in comparison to the
 * cuda texture objects.
 * @param x
 * @param y
 * @param surface
 * @param width
 * @param height
 * @return
 */
// TODO: adapt this for texture atlas
__device__ 
inline float4 readBilinear(float x, float y, const cudaSurfaceObject_t surface, 
                           int width, int height) {

	if(x < 0 || y < 0 || x > (width - 1) || y > (height - 1)) {
		if(x == 0 && blockIdx.x == 0) {
			//printf("shit are we out of bounds? %f of %d \n",y,height);
		}
		return make_float4(NAN, NAN, NAN, NAN);
	}
	float x_tmp = x - float(int(x));
	float y_tmp = y - float(int(y));

	//in the edge case of being on the upper bound we change the composition of the
	//pixel coordinates a little
	//changing from [) to [] .... you know what i mean. (hopefully)
	if(x_tmp == 0 && int(x) == width - 1) {
		x_tmp = 1.0f;
		x = x - 1.0f;
	}

	if(y_tmp == 0 && int(y) == height - 1) {
		y_tmp = 1.0f;
		y = y - 1.0f;
	}

	//bottom:
	float4 Q00;
	surf2Dread(&Q00, surface, int(x) * sizeof(float4), int(y));
	float4 Q01;
	surf2Dread(&Q01, surface, (int(x) + 1) * sizeof(float4), int(y));
	float4 Q0x = x_tmp * Q01 + (1.0f - x_tmp) * Q00;

	//top
	float4 Q10;
	surf2Dread(&Q10, surface, int(x) * sizeof(float4), int(y) + 1);
	float4 Q11;
	surf2Dread(&Q11, surface, (int(x) + 1) * sizeof(float4), int(y) + 1);
	float4 Q1x = x_tmp * Q11 + (1.0f - x_tmp) * Q10;

	//mix the middle;
	float4 Qyx = y_tmp * Q1x + (1.0f - y_tmp) * Q0x;

	return Qyx;
}

__device__ 
inline float4 readFloat4F16(const cudaSurfaceObject_t surface, int x, int y) {
	__half data[4];
	surf2Dread((ushort4*) data, surface, x * sizeof(ushort4), y);
	float4 output = make_float4(__half2float(data[0]),
	                            __half2float(data[1]),
	                            __half2float(data[2]),
	                            __half2float(data[3]));
	return output;
}

//TODO: adapt this for texture atlas
__device__ 
inline float4 readBilinear16F(float x, float y, 
                              const cudaSurfaceObject_t surface, 
                              int width, int height) {

	if(x < 0 || y < 0 || x > (width - 1) || y > (height - 1)) {
		if(x == 0 && blockIdx.x == 0) {
			//printf("shit are we out of bounds? %f of %d \n",y,height);
		}
		return make_float4(NAN, NAN, NAN, NAN);
	}
	float x_tmp = x - float(int(x));
	float y_tmp = y - float(int(y));

	//in the edge case of being on the upper bound we change the composition of the
	//pixel coordinates a little
	//changing from [) to [] .... you know what i mean. (hopefully)
	if(x_tmp == 0 && int(x) == width - 1) {
		x_tmp = 1.0f;
		x = x - 1.0f;
	}

	if(y_tmp == 0 && int(y) == height - 1) {
		y_tmp = 1.0f;
		y = y - 1.0f;
	}

	//bottom:
	float4 Q00 = readFloat4F16(surface, x, y);
	float4 Q01 = readFloat4F16(surface, x + 1, y);
	float4 Q0x = x_tmp * Q01 + (1.0f - x_tmp) * Q00;

	//top
	float4 Q10 = readFloat4F16(surface, x, y + 1);
	float4 Q11 = readFloat4F16(surface, x + 1, y + 1);
	float4 Q1x = x_tmp * Q11 + (1.0f - x_tmp) * Q10;

	//mix the middle;
	float4 Qyx = y_tmp * Q1x + (1.0f - y_tmp) * Q0x;

	return Qyx;
}

/**
 * @brief readBilinearGLStyle
 * opengl style texture sampling is using normalized coordinates. The pixel centers are
 * @param x
 * @param y
 * @param surface
 * @param width
 * @param height
 * @return
 */
//TODO: adapt this for the texture atlas
__device__ 
inline float4 readBilinearGLStyle(float x, float y, 
                                  const cudaSurfaceObject_t surface,
                                  int width, int height) {
	x = x * float(width - 1) - 0.5f;
	y = y * float(height - 1) - 0.5f;

	//TODO: do some simple thresholding
	x = fmin(fmax(x, 0), width - 1);
	y = fmin(fmax(y, 0), height - 1);
	return readBilinear(x, y, surface, width, height);
}

__device__ 
inline float4 calcSurfaceUpdate(float4 surface_k,float4 sensor, //the vector of sensor data and of what is on the surface
                                float d, float d_up) {
	float epsilon = 0.001; // if sigma comes this close to
	float dev_k = surface_k.x;

	//deviation as it is perceived to this surface right now
	float dev_s = d - sensor.x;

	//the minimum possible standard deviation
	float s_m = min(sensor.z, surface_k.z);
	float s_k = surface_k.y;
	float s_s = sensor.y;

	//standard deviation to the lower bound of standard deviation
	float s_s_ = s_s - s_m;
	float s_k_ = s_k - s_m;

	float s_k1_ = s_k_ * s_s_ / (s_k_ + s_s_);
	if(s_k1_ < epsilon) {
		s_k1_ = 0; //set the standard deviation update
		           //to zero if it is too close to zero
	}
	float dev_k1 = (dev_s / s_s + dev_k / s_k_) * s_k1_;

	float4 surface_k1;
	surface_k1.x = dev_k1 - (d - d_up);
	surface_k1.y = s_k1_ + s_m;
	surface_k1.z = s_m;
	surface_k1.w = surface_k.w;

	return surface_k1;
}

#endif

#endif // FILE_SURFACE_READ_H
