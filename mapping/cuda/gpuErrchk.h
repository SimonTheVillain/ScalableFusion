#ifndef FILE_CUDA_ERRCHK_H
#define FILE_CUDA_ERRCHK_H
#include <stdio.h>
#include <assert.h>
#include <cublas.h>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess)
   {
      const char* errorString = cudaGetErrorString(code);
      fprintf(stderr,"GPUassert: %s %s %d\n", errorString, file, line);
      if (abort) assert(false);
   }
}

#endif
