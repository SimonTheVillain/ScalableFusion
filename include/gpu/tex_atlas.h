#ifndef FILE_TEX_ATLAS_H
#define FILE_TEX_ATLAS_H

#include <memory>
#include <mutex>
#include <stack>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gpu/gpu_tex.h>
#include <cuda/gpu_mesh_structure.h>
#include <gpu/thread_safe_FBO_VAO.h>

using namespace std;

class TexAtlas;
class TexAtlasTex;
class TexAtlasPatch;
class GarbageCollector;

class TexAtlas {
public:

	//probably don't need cv type
	TexAtlas(GarbageCollector *garbage_collector, GLuint int_type, GLuint type, 
	         GLuint format, int cv_type, int res = 1024, 
	         ThreadSafeFBOStorage *fbos = nullptr);//create one texture atlas for every type of

	~TexAtlas();

	shared_ptr<TexAtlasPatch> getTexAtlasPatch(cv::Size2i size);

	size_t getMaxRes() {
		return max_res_;
	}

	int getCvType() {
		return cv_type_;
	}

	GLuint getGlIntType() {
		return int_type_;
	}

	GLuint getGlType() {
		return type_;
	}

	int countPatches();

	int countTex();

private:

	mutex mutex_;

	//vector of vector with sizes
	struct SafeVector{
		mutex tex_mutex;
		vector<weak_ptr<TexAtlasTex>> tex;
	};

	SafeVector *textures_;
	GLuint int_type_;
	GLuint format_;
	GLuint type_;
	int cv_type_;
	int max_res_;
	ThreadSafeFBOStorage *fbo_storage_;
	GarbageCollector *garbage_collector_;

	#ifdef GL_MEMORY_LEAK_WORKAROUND
	vector<shared_ptr<TexAtlasTex>> tex_retainer_;
	#endif

};

class TexAtlasTex {
	friend TexAtlasPatch;
	friend TexAtlas;

public:

	~TexAtlasTex();

	bool hasSlot();

	void showImage(string text);

	void showImage(string text, cv::Rect2i cut_out);

	GLuint getFBO();

	shared_ptr<gfx::GpuTex2D> getTex2D() {
		return tex_;
	}
	int getCvType() {
		return cv_type_;
	}

	int countPatches();
	int cvType() { return cv_type_;}

	mutex tex_atlas_tex_mutex;
	
private:

	//TODO: make the constructor sort of private
	TexAtlasTex(GarbageCollector *garbage_collector, GLuint int_type, GLuint type,
	            GLuint format, int cv_type, int res,int full_res,
	            ThreadSafeFBOStorage *fbo_storage);

	cv::Rect2i posFromIndex_(int i);

	//TODO: let this throw an exception if there is no slot left
	shared_ptr<TexAtlasPatch> getFreePatch_(shared_ptr<TexAtlasTex> self);

	void freeTexSlot_(int in_slot);

	shared_ptr<gfx::GpuTex2D> tex_;
	int tile_size_; //8,16,32,64,etc.
	int cv_type_;
	mutex occupants_mutex_;

	//TODO: this is genious... the stack tells us if and where there is place for new textures
	//all of this while being superfast. (has to be filled at constructor with empty slots)
	//TODO: also implement a system like this for the GPU GEOMETRY STORAGE)
	stack<int> free_slots_;

	//GLuint FBO=0;
	ThreadSafeFBO *fbo_ = nullptr;
	bool debug_ = false;

	thread::id debug_thread_id_tex_created_in_;

};

class TexAtlasPatch {
	friend TexAtlasTex;
	friend TexAtlas;

public:

	~TexAtlasPatch();

	shared_ptr<gfx::GpuTex2D> getTex();

	cv::Rect2i getPosition();

	cv::Rect2i getRect() {
		return cv::Rect(pos_.x, pos_.y, size_.width, size_.height);
	}

	shared_ptr<TexAtlasTex> getAtlasTex() {
		return tex_;
	}

	int getFramebuffer();

	void setFramebufferActive();

	GLuint getFBO();

	void setViewport();

	void setViewport(cv::Size2i size);

	bool downloadData(void *data,cv::Rect2i rect);

	bool downloadData(void *data, cv::Size2i size) {
		return 
				downloadData(data, cv::Rect2i(pos_.x, pos_.y, size.width, size.height));
	}

	bool downloadAllData(void *data) {
		return downloadData(data, pos_);
	}

	bool downloadData(void *data) {
		return downloadData(data, getRect());
	}

	bool uploadData(void *data, cv::Rect2i rect);

	bool uploadData(void *data, cv::Size2i size) {
		return uploadData(data, cv::Rect2i(pos_.x, pos_.y, size.width, size.height));
	}

	bool uploadAllData(void *data) {
		return uploadData(data, pos_);
	}
	bool uploadData(void *data) {
		return uploadData(data, getRect());
	}

	cudaSurfaceObject_t getCudaSurfaceObject();
	cudaTextureObject_t getCudaTextureObject();
	uint64_t getGlHandle();

	void showImage(string text);

	void showImage(string text, cv::Size2i size);

	GpuTextureInfo genTexInfo(cv::Size2i size, int tex_coord_starting_index);

	GpuTextureInfo genTexInfo(int tex_coord_starting_index);

	//test if this patch is required to
	bool isGpuResidencyRequired();

	int cvType() {return tex_->cvType();}

private:
	
	TexAtlasPatch(shared_ptr<TexAtlasTex> &tex, cv::Rect2i &pos, int index);

	shared_ptr<TexAtlasTex> tex_;
	cv::Rect2i pos_;//position and reserved size within the texture atlas
	cv::Size2i size_;//size which is actually used
	int index_in_atlas_tex_;

};

#endif // FILE_TEX_ATLAS_H
