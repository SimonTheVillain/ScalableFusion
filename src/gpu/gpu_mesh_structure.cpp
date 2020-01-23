//
// Created by simon on 06.12.19.
//

#include <gpu/gpu_mesh_structure.h>
#include <base/mesh_structure.h>
#include <cuda/float16_utils.h>
#include <cuda/gpu_errchk.h>


GpuTextureInfo TextureLayerGPU::genGpuTextureInfo(){
	GpuTextureInfo info;
	info = tex->genTexInfo(coords->getStartingIndex());
	return info;
}

MeshletGPU::~MeshletGPU(){

	if(vertex_token == nullptr)
		return;

	shared_ptr<Meshlet> meshlet = vertex_token->lock();
	if(meshlet == nullptr)
		return; //the meshlet 	we would have stored back the data doesn't exist anymore

	meshlet->vertices_mutex.lock(); //TODO: maybe make this a shared mutex so the vertices can be read nonetheless

	if(meshlet->triangles_version > triangle_version){
		return;//the meshlet we were to save back is more up to date than we would like
	}else if(meshlet->triangles_version < triangle_version){
		assert(0);//this would mean somebody updated the mesh on the GPU (should not happen)
	}

	if(meshlet->vertices_version > vertex_version)
		assert(0); // this would mean somebody updated vertex positions on the CPU (should not happen)

	vector<GpuVertex> gpu_verts(meshlet->vertices.size());
	vertices->download(&gpu_verts[0],0,meshlet->vertices.size());//only download the vertices of interest

	for(int i=0;i<meshlet->vertices.size();i++){
		meshlet->vertices[i].p = gpu_verts[i].p;
		meshlet->vertices[i].n = gpu_verts[i].n;
	}
	meshlet->vertices_version = vertex_version;
	meshlet->vertices_mutex.unlock();


	//TODO: remove everything needed to transcribe neighbouring vertex positions to this patch!!!


	if(gpu_vert_transcribe_tasks != nullptr){
		cudaFree(gpu_vert_transcribe_tasks);
		gpu_vert_transcribe_tasks = nullptr;
		gpu_vert_transcribe_task_count = 0;
	}
	if(gpu_neighbour_vertices != nullptr){
		cudaFree(gpu_neighbour_vertices );
		gpu_neighbour_vertices = nullptr;
	}


}

TextureLayerGPU::~TextureLayerGPU(){
	if(tex == nullptr)
		return;
	if(coords == nullptr)
		return;
	if(token == nullptr)
		return;
	shared_ptr<MeshTexture> cpu_tex = token->lock();
	if(cpu_tex == nullptr)
		return;

	cv::Rect2i rect = tex->getRect();
	int internal_format = tex->getTex()->getGlInternalFormat();
	cv::Mat data;
	if(		internal_format == GL_RGBA16F||
			   internal_format == GL_RGB16F ||
			   internal_format == GL_RG16F  ||
			   internal_format == GL_R16F) {
		int channels = tex->getTex()->getChannelCount();
		/*
		uint8_t* data_gpu;
		size_t byte_count = sizeof(float) * rect.width*rect.height * channels;
		cudaMalloc(&data_gpu,byte_count);

		castF16SurfaceToF32Buffer(tex->getTex()->getCudaSurfaceObject(),
								  rect.x,rect.y,rect.width,rect.height,
								  (float*)data_gpu,
								  tex->getTex()->getChannelCount());

		data = cv::Mat(rect.height,rect.width,CV_32FC(channels));
		cudaMemcpy(data.data,data_gpu,byte_count,cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize();
		cv::imshow("downloaded std_tex_patch",data);
		cv::waitKey(1);
		//assert(0);//if this happens the first few frames something is really off
		cudaFree(data_gpu);
		*/

		data = cv::Mat(rect.height,rect.width,CV_16UC(channels));
		tex->downloadData(data.data);


	}else{
		//assert(0);
		data = cv::Mat(rect.height,rect.width,tex->cvType());
		tex->downloadData(data.data);
	}
	//TODO: something about a version
	vector<Vector2f> tex_coords(coords->getSize());
	coords->download(&tex_coords[0]);

	cpu_tex->mat_mutex.lock();
	cpu_tex->mat = data;
	cpu_tex->tex_coords = tex_coords;
	cpu_tex->tex_coord_version = tex_coord_version;
	cpu_tex->tex_version = tex_version;
	cpu_tex->mat_mutex.unlock();





	//assert(0);
}




void TextureLayerGPU::create(shared_ptr<MeshTexture> cpu_texture, TexAtlas* tex_atlas,GpuBuffer<Eigen::Vector2f>* coord_buffer) {


	//upload the image
	cv::Mat data = cpu_texture->mat;
	cv::Size2i size = data.size();
	shared_ptr<TexAtlasPatch> patch = tex_atlas->getTexAtlasPatch(size);
	int internal_format = patch->getTex()->getGlInternalFormat();
	if(		internal_format == GL_RGBA16F||
			   internal_format == GL_RGB16F ||
			   internal_format == GL_RG16F  ||
			   internal_format == GL_R16F) {

		//int byte_count = patch->getTex()->getChannelCount() * sizeof(float) * size.width * size.height;
		//uint8_t* data_gpu;
		//cudaMalloc(&data_gpu,byte_count);
		//cudaMemcpy(data_gpu,data.data,byte_count,cudaMemcpyHostToDevice);
		patch->uploadData(data.data);
		/*
		cv::imshow("upload std_tex_patch",data);
		cv::waitKey(1);

		cv::Rect2i rect = patch->getRect();
		castF32CPUBufferToF16Surface(patch->getTex()->getCudaSurfaceObject(),
								  rect.x,rect.y,rect.width,rect.height,
								  (float*)data.data,
								  patch->getTex()->getChannelCount());
		//cudaFree(data_gpu);
		cudaDeviceSynchronize();//just for debug!!!
		gpuErrchk(cudaPeekAtLastError());


		//TODO: remove everything below this!!!!!!!
		//DEBUG test if what we uploaded can be downloaded as well
		uint8_t* data_gpu2;
		rect = patch->getRect();
		int channels = patch->getTex()->getChannelCount();
		size_t byte_count2 = sizeof(float) * rect.width*rect.height * channels;
		cudaMalloc(&data_gpu2,byte_count2);

		castF16SurfaceToF32Buffer(patch->getTex()->getCudaSurfaceObject(),
								  rect.x,rect.y,rect.width,rect.height,
								  (float*)data_gpu2,
								  patch->getTex()->getChannelCount());

		data = cv::Mat(rect.height,rect.width,CV_32FC(channels));
		cudaMemcpy(data.data,data_gpu2,byte_count2,cudaMemcpyDeviceToHost);
		cudaDeviceSynchronize();
		cv::imshow("Re-downloaded std_tex_patch",data);
		cv::waitKey();
		//assert(0);//if this happens the first few frames something is really off
		cudaFree(data_gpu2);

		 */
		cudaDeviceSynchronize();//just for debug!!!
		gpuErrchk(cudaPeekAtLastError());

	}else{
		patch->uploadData(data.data);
	}


	//upload the tex coordinates
	shared_ptr<GpuBufferConnector<Vector2f>> buffer = coord_buffer->getBlock(cpu_texture->tex_coords.size());
	buffer->upload(&cpu_texture->tex_coords[0]);

	//set the members
	tex = patch;
	coords = buffer;
	tex_coord_version = cpu_texture->tex_coord_version;
	tex_version = cpu_texture->tex_version;

	token = make_unique<weak_ptr<MeshTexture>>(cpu_texture);
	//weak_ptr<MeshTexture> weak_cpu_tex = cpu_texture;
	//token = make_unique<weak_ptr<MeshTexture>>(weak_cpu_tex);

	//create vertex buffer and upload to that as well
	//upload float32 to float16 (depending on the datatype)
	//patch->uploadData();

	//assert(0);
}