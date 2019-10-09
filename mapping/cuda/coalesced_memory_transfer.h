#ifndef FILE_COALESCED_MEMORY_TRANSFER_H
#define FILE_COALESCED_MEMORY_TRANSFER_H

#include <vector>

#include "gpu_mesh_structure.h"
//it might be beneficial to combine downloads and then do the copying on the cpu side.

using namespace std;

//especially when downloading vertex data.
void downloadVertices(vector<GpuVertex*> gpuVertices, GpuVertex *data);

class CoalescedGpuTransfer {
public:

	struct Task {
		int  start;
		int  count;
		void *target;
	};

	struct TaskD2D {
		size_t source_index;
		size_t destination_index;
		size_t count;
	};

	template<typename T>
	struct SetTaskTemp {
		T *dst;
		T value;
	};

	template<typename T>
	struct CpyTaskTemp {
		T *src;
		T *dst;
	};

	struct DirectTask {
		void *src;
		void *dst;
		int  byteCount;
	};

	template<typename T>
	static void upload(vector<T> source, vector<Task> tasks);

	// This is especially useful for the header information
	template<typename T>
	static void upload(vector<T> source, vector<T*> gpu_dst);

	//needed, e.g. when copying texture coordinates
	template<typename T>
	static void device2DeviceSameBuf(T *buffer, vector<TaskD2D> tasks);

	template<typename T>
	static void upload(vector<SetTaskTemp<T>> tasks);

	template<typename T>
	static void copy(vector<CpyTaskTemp<T>> tasks);

	static void download(vector<DirectTask> tasks);

};

#endif // FILE_COALESCED_MEMORY_TRANSFER_H
