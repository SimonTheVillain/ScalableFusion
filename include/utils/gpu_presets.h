#ifndef FILE_GPU_PRESETS_H
#define FILE_GPU_PRESETS_H

#include <map>
#include <string>

class GpuConf {
public:
	//all possible parameters:
	int vertex_buf_size;
	int triangle_buf_size;
	int patch_header_buf_size;
}

//there should be a static map with all the possible gpu presets
//thus we can initialize our algorithms correctly
class GpuPresets {
public:
	GpuConf getParams(std::string gpu);
};

#endif // FILE_GPU_PRESETS_H
