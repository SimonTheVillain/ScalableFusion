#ifndef FILE_TEST_H
#define FILE_TEST_H

#include <cuda.h>

#include <cublas.h>

#include <opencv2/opencv.hpp>
//#include <opencv2/cudaimgproc.hpp>
//#include <opencv2/cudawarping.hpp>


//TODO: small code example to copy (opengl) bindless texture to cv::cuda::GpuMat
#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) assert(false);
   }
}

void copy(cudaTextureObject_t texture,cv::cuda::GpuMat &to);
#endif
