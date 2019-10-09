#ifndef FILE_GPU_ERRCHK_H
#define FILE_GPU_ERRCHK_H

#include <stdio.h>
#include <assert.h>
#include <cublas.h>

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, 
                      bool abort = true) {
   if (code != cudaSuccess) {
      const char *error_string = cudaGetErrorString(code);
      fprintf(stderr, "GPUassert: %s %s %d\n", error_string, file, line);
      if(abort) 
      	assert(false);
   }
}

#endif // FILE_GPU_ERRCHK_H
