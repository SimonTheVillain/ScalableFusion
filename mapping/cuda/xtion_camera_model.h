#ifndef FILE_XTION_CAMERA_MODEL_H
#define FILE_XTION_CAMERA_MODEL_H

//given the depthmap of the xtion the camera model returns the standard deviation of the signal,
//the minimum achievable standard deviation at this distance and the depth combined within one texture.

//#define OVERSIMPLIFY_CAM_MODEL

void generateXtionConfidenceImage(const cudaSurfaceObject_t raw_depth,
                                  cudaSurfaceObject_t output,
                                  int width, int height);


void generateXtionConfidenceImage16F(const cudaSurfaceObject_t raw_depth,
                                     cudaSurfaceObject_t output,
                                     int width, int height);

#ifdef __CUDACC__
__device__
#endif // __CUDACC__
inline float xtionStdToThresholdSeg(float std) {
	#ifdef OVERSIMPLIFY_CAM_MODEL
	//return std; //(DEBUG) this should always be 0.05
	return 0.05;
	#endif // OVERSIMPLIFY_CAM_MODEL
	return std * 1.2;
}

//the idea is that we want differentiate between the thresholds used to create the
//threshold for creating new geometry and adding geometry
//UNUSED
inline float xtionStdToThresholdCreate(float std) {
	return std * 0.9;
}

#endif // FILE_XTION_CAMERA_MODEL_H
