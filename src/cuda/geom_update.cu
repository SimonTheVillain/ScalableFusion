#include <cuda/geom_update.h>

#include <cuda/vertex_update.h>
#include <cuda/std_tex_update.h>
#include <cuda/surface_read.h>
#include <cuda/xtion_camera_model.h>

//TODO: REMOVE DEBUG
#include <iostream>

using namespace std;
using namespace Eigen;

//We only should do a texture update for patches whose all neighbour patches are loaded.
//this means we do the vertex update for all patches that are loaded, but only do the textre update
//where all the neighbouring patches are present
//TODO: get rid of this integer debug value
int gpu::updateGeometry(const cudaSurfaceObject_t geometry_input, //the sensor input adapted by standard deviations
                        int width, int height, //sensor resolution
                        const vector<gpu::UpdateDescriptor> &descriptors,
                        Vector4f cam_pos,
                        Matrix4f pose, // because we want the vertex position relative to the camera
                        Matrix4f proj_pose, //to get the position of the point on the image.
                        GpuVertex *vertices, Vector2f *tex_pos,
                        GpuTriangle *triangles, GpuPatchInfo *patch_infos) { //pointer to the geometric data

	if(descriptors.empty()) {
		return-1;
	}

	dim3 block(256);// using 1024); works on desktops but it is killing the tegra
	dim3 grid(descriptors.size());
	gpu::UpdateDescriptor *descs;
	cudaMalloc(&descs, descriptors.size() * sizeof(gpu::UpdateDescriptor));
	cudaMemcpy(descs, &descriptors[0], 
	           descriptors.size() * sizeof(gpu::UpdateDescriptor), 
	           cudaMemcpyHostToDevice);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());

	//first we update the vertices
	vertexUpdate_kernel<<<grid, block>>>(geometry_input, //the sensor input adapted by standard deviations
	                                     width, height, //sensor resolution
	                                     descs,
	                                     cam_pos,
	                                     pose, // because we want the vertex position relative to the camera
	                                     proj_pose, //to get the position of the point on the image.
	                                     vertices, tex_pos,
	                                     triangles, patch_infos);

	cudaDeviceSynchronize();//just for debug!!!
	gpuErrchk(cudaPeekAtLastError());

	updateGeomTex_kernel<<<grid, block>>>(geometry_input, //the sensor input adapted by standard deviations
	                                      width, height, //sensor resolution
	                                      descs,
	                                      cam_pos,
	                                      pose, // because we want the vertex position relative to the camera
	                                      proj_pose, //to get the position of the point on the image.
	                                      vertices, tex_pos,
	                                      triangles, patch_infos);

	//then we update the texture.
	//why is this crashing????

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
	cudaFree(descs);

	return -1;
}

__global__
void checkVertexValidity_kernel(const cudaSurfaceObject_t sensor_image,
                                int width, int height,
                                Matrix4f pose, Matrix4f proj_pose,
                                gpu::GeometryValidityChecks::VertexTask *tasks,
                                GpuVertex *vertices) {

	uint32_t k = blockIdx.x;
	gpu::GeometryValidityChecks::VertexTask task = tasks[k];

	uint32_t i = threadIdx.x;
	while(i < task.size) {
		GpuVertex vertex = vertices[i + task.start_source];

		//TODO: all the logic!!!!!
		//calculate position on the sensor
		Vector4f p = vertex.p;
		p[3] = 1;
		Vector4f p_cam = pose * p;
		float z = p_cam[2];

		Vector4f pos_on_sensor = proj_pose * p;

		float u = pos_on_sensor[0] / pos_on_sensor[3];
		float v = pos_on_sensor[1] / pos_on_sensor[3];

		float4 sensor = readSensor(u, v, sensor_image, width, height, 0.05); //threshold =0.1

		float threshold = xtionStdToThresholdSeg(sensor.y);//the second one is the surface

		if(z < (sensor.x - threshold) && !isnan(sensor.x)) {
			//invalidate vertex
			//for debug purposes do it in the source
			//TODO: do it at destRect
			vertices[i + task.start_source].valid = 0;
		}

		vertices[i + task.start_dest] = vertex;
		i += blockDim.x;
	}
}

void gpu::GeometryValidityChecks::checkVertexValidity(
		const cudaSurfaceObject_t sensor, int width, int height, Matrix4f pose,
		Matrix4f proj_pose, vector<GeometryValidityChecks::VertexTask> tasks,
		GpuVertex *vertices) {

	if(tasks.empty()) {
		return;
	}

	dim3 block(256);
	dim3 grid(tasks.size());
	gpu::GeometryValidityChecks::VertexTask *gpu_tasks;

	cudaMalloc(&gpu_tasks,
	           sizeof(gpu::GeometryValidityChecks::VertexTask) * tasks.size());

	cudaMemcpy(gpu_tasks, &(tasks[0]),
	           sizeof(gpu::GeometryValidityChecks::VertexTask) * tasks.size(),
	           cudaMemcpyHostToDevice);

	checkVertexValidity_kernel<<<grid, block>>>(sensor, width, height, pose,
	                                            proj_pose, gpu_tasks, vertices);

	cudaFree(gpu_tasks);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}

__global__
void checkTriangleValidity_kernel(
		gpu::GeometryValidityChecks::TriangleTask *tasks, 
		const cudaSurfaceObject_t sensor, int width, int height, Matrix4f pose,
		Matrix4f proj_pose, GpuVertex *vertices, Vector2f *tex_pos, 
		GpuTriangle *triangles, GpuPatchInfo *patch_infos) {

}

void gpu::GeometryUpdate::calcCenterAndRadiusKernelCall_(
		dim3 grid, dim3 block, size_t bytes, CalcCenterTask *gpu_tasks,
		Vector4f *results) {
    cudaDeviceSynchronize(); //TODO: REMOVE THESE DEBUG SYNCHRONIZATIONS!!!!
    gpuErrchk(cudaPeekAtLastError());
    std::cout << "before running the calcCenter_kernel grid: " << grid.x << "block: " << block.x << std::endl;
	calcCenter_kernel<<<grid, block, bytes>>>(gpu_tasks, results);
    cout << "after running the calcCenter_kernel" << endl;
    cudaDeviceSynchronize();
    gpuErrchk(cudaPeekAtLastError());
    cout << "after running the calcCenter_kernel synchronization" << endl;
}

void gpu::GeometryValidityChecks::checkTriangleValidity(
		vector<TriangleTask> tasks, const cudaSurfaceObject_t sensor,
		int width, int height, Matrix4f pose, Matrix4f proj_pose,
		GpuVertex *vertices, Vector2f *tex_pos, GpuTriangle *triangles,
		GpuPatchInfo *patch_infos) {

	assert(0); //this is not a priority yet.
	if(tasks.empty()) {
		return;
	}
	dim3 block(256);
	dim3 grid(tasks.size());
	GeometryValidityChecks::TriangleTask *gpu_tasks;

	cudaMalloc(&gpu_tasks, 
	           sizeof(GeometryValidityChecks::TriangleTask) * tasks.size());

	cudaMemcpy(gpu_tasks, &(tasks[0]),
	           sizeof(GeometryValidityChecks::TriangleTask) * tasks.size(),
	           cudaMemcpyHostToDevice);

	checkTriangleValidity_kernel<<<grid, block>>>(gpu_tasks, sensor, width, 
	                                              height, pose, proj_pose,
	                                              vertices, tex_pos, triangles,
	                                              patch_infos);

	cudaFree(gpu_tasks);

	cudaDeviceSynchronize();
	gpuErrchk(cudaPeekAtLastError());
}
