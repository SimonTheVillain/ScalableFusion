#include "tex_atlas.h"

#include <gfx/gl_utils.h>
#include <gfx/garbage_collector.h>

using namespace std;
using namespace Eigen;

TexAtlas::TexAtlas(GarbageCollector *garbage_collector, GLuint int_type, 
                   GLuint type, GLuint format, int cv_type, int res, 
                   ThreadSafeFBOStorage *fbos) 
		: garbage_collector_(garbage_collector),
		  int_type_(int_type),
		  type_(type),
		  format_(format),
		  cv_type_(cv_type),
		  max_res_(res),
		  textures_(new SafeVector[(int)ceil(log2(float(res))) + 1]()),
		  fbo_storage_(fbos) {

}

TexAtlas::~TexAtlas() {
	delete[] textures_;
}

shared_ptr<TexAtlasPatch> TexAtlas::getTexAtlasPatch(cv::Size2i size) {
	float max_dim = max(size.width, size.height);
	if(max_dim > max_res_) {
		cout << "Texture exceeds maximum allowed size in this atlas structure" << endl;
		assert(0);
		throw;
	}
	int s = ceil(log2(float(max_dim)));

	#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
	cout << "DEBUG: requested " << size <<", delivered " << pow(2, s) << endl;
	cout << (int) ceil(log2(float(1024))) << endl;
	#endif

	int target_size = pow(2,s);
	shared_ptr<TexAtlasPatch> found_tex_patch = nullptr;
	SafeVector &tex_this_size = textures_[s];
	tex_this_size.tex_mutex.lock();
	tex_this_size.tex.erase(remove_if(
			tex_this_size.tex.begin(), tex_this_size.tex.end(), 
			[](weak_ptr<TexAtlasTex> unlocked) {return unlocked.lock() == nullptr;}),
			tex_this_size.tex.end());
	for(size_t i = 0; i < tex_this_size.tex.size(); i++) {
		//check the existing textures if there is some place left to store data onto
		//

		//if the TexAtlasTex is valid we want to secure it in case somebody wants to delete it,
		//this is done by just letting a shared_ptr point to it
		shared_ptr<TexAtlasTex> locked = tex_this_size.tex[i].lock();
		if(locked == nullptr) {
			assert(0);//we filtered this before (the algorithm below would jump over the next possible entry
			//if the texture has expired because no texture patch was pointing at it anymore
			//we delete the dead reference in our list
			tex_this_size.tex.erase(tex_this_size.tex.begin() + i);//TODO: we are not fully iterating anymore
			continue;
		}

		found_tex_patch = locked->getFreePatch_(locked);
		if(found_tex_patch != nullptr) {
			found_tex_patch->size_ = size;
			tex_this_size.tex_mutex.unlock();
			return found_tex_patch;
		}//else just continue searching

	}
	if(found_tex_patch == nullptr) {
		#ifdef SHOW_SERIOUS_DEBUG_OUTPUTS
		cout << "DEBUG: since there is no free texture left we create a new texture" << endl;
 		#endif
		//we obviously don't have any free patches left in this texture
		//so we create a new one
		shared_ptr<TexAtlasTex> tex = shared_ptr<TexAtlasTex>(
				new TexAtlasTex(garbage_collector_, int_type_, type_, format_, cv_type_,
				target_size, max_res_, fbo_storage_));
		found_tex_patch = tex->getFreePatch_(tex);
		tex_this_size.tex.push_back(tex);
		#ifdef GL_MEMORY_LEAK_WORKAROUND
		tex_retainer_.push_back(tex);
		#endif
	}
	found_tex_patch->size_ = size;
	tex_this_size.tex_mutex.unlock();
	return found_tex_patch;
}

int TexAtlas::countPatches() {
	int count = 0;
	size_t sizes = ceil(log2(max_res_));
	for(size_t i = 0; i < sizes; i++) {
		SafeVector &tex_of_size = textures_[i];
		tex_of_size.tex_mutex.lock();
		for(size_t j = 0; j < tex_of_size.tex.size(); j++) {
			shared_ptr<TexAtlasTex> tex = tex_of_size.tex[j].lock();
			if(tex == nullptr) {
				continue;
			}
			count += tex->countPatches();
		}
		tex_of_size.tex_mutex.unlock();
	}
	return count;
}

int TexAtlas::countTex() {
	int count = 0;
	size_t sizes = ceil(log2(max_res_));
	for(size_t i = 0; i < sizes; i++) {
		SafeVector &tex_of_size = textures_[i];
		tex_of_size.tex_mutex.lock();
		for(size_t i = 0; i < tex_of_size.tex.size(); i++) {
			if(!tex_of_size.tex[i].expired()) {
				count++;
			}
		}
		tex_of_size.tex_mutex.unlock();
	}
	return count;
}

cv::Rect2i TexAtlasTex::posFromIndex_(int i) {
	int tiles_per_side = tex_->getWidth() / tile_size_;
	int y = (i / tiles_per_side) * tile_size_;
	int x = (i % tiles_per_side) * tile_size_;
	return cv::Rect2i(x, y, tile_size_, tile_size_);
}

shared_ptr<TexAtlasPatch> TexAtlasTex::getFreePatch_(
		shared_ptr<TexAtlasTex> self) {
	occupants_mutex_.lock();
	if(free_slots_.empty()) {
		occupants_mutex_.unlock();
		return nullptr;
	}
	int i = free_slots_.top();
	free_slots_.pop();

	cv::Rect2i r = posFromIndex_(i);
	shared_ptr<TexAtlasPatch> patch = shared_ptr<TexAtlasPatch>(
			new TexAtlasPatch(self, r, i));
	occupants_mutex_.unlock();
	return patch;
}

void TexAtlasTex::freeTexSlot_(int in_slot) {
	occupants_mutex_.lock();
	free_slots_.push(in_slot);
	occupants_mutex_.unlock();
}

TexAtlasTex::TexAtlasTex(GarbageCollector *garbage_collector, GLuint int_type,
                         GLuint type, GLuint format, int cv_type, int res, 
                         int full_res, ThreadSafeFBOStorage *fbo_storage) {
	debug_ = false;
	debug_thread_id_tex_created_in_ = this_thread::get_id();

	tex_ = make_shared<gfx::GpuTex2D>(garbage_collector, int_type, format, type,
	                                 full_res, full_res, true, nullptr);
	glFinish();//force the texture to really be created... (i ho
	debug_ = true;
	cv_type_ = cv_type;
	tile_size_ = res;
	int s = full_res / res;
	for(size_t i = 0; i < s * s; i++) {
		free_slots_.push(i);
	}
	fbo_ = fbo_storage->createGlObject();
}

TexAtlasTex::~TexAtlasTex() {
	if(fbo_ != nullptr) {
		delete fbo_;
	}
}

void TexAtlasTex::showImage(string text) {
	showImage(text, cv::Rect2i(0, 0, tile_size_, tile_size_));
}

void TexAtlasTex::showImage(string text, cv::Rect2i cut_out) {
	cv::Mat image(cut_out.height, cut_out.width, cv_type_);//TODO: the opencv format is just a placeholder
	tex_->downloadData(image.data, cut_out.x, cut_out.y, cut_out.width,
	                   cut_out.height);
	imshow(text, image);
}

GLuint TexAtlasTex::getFBO() {
	if(!fbo_->existsInThisThread()) {
		//making sure the texture really is created.... shit. i really hate this
		glBindFramebuffer(GL_FRAMEBUFFER, fbo_->get());
		glBindTexture(GL_TEXTURE_2D, tex_->getGlName());//not necessary
		glFramebufferTexture(GL_FRAMEBUFFER, 
		                     GL_COLOR_ATTACHMENT0, tex_->getGlName(), 0);
		GLenum draw_buffers[1] = {GL_COLOR_ATTACHMENT0};
		glDrawBuffers(1, draw_buffers);

		GLenum gl_err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if(gl_err != GL_FRAMEBUFFER_COMPLETE) {
			cout << "debug variable set?" << debug_ << endl;
			glBindTexture(GL_TEXTURE_2D, tex_->getGlName());
			GLint result;
			glGetTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_RESIDENT, &result);
			cout << "is texture resident" << result << endl;
			//do even more debug stuff:
			GLuint desperate_fbo;
			glGenFramebuffers(1, &desperate_fbo);
			glBindFramebuffer(GL_FRAMEBUFFER, desperate_fbo);
			glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
			                     tex_->getGlName(), 0);
			GLenum draw_buffers[1] = {GL_COLOR_ATTACHMENT0};
			glDrawBuffers(1, draw_buffers);
			//if after this, the error still occurs it probably is on the
			//texture and not on the framebuffer itself.
			gfx::GLUtils::checkOpenGLFramebufferStatus("TexAtlasTex::getFBO");
			assert(0);
		}
		thread::id id = this_thread::get_id();
		gfx::GLUtils::checkOpenGLFramebufferStatus("TexAtlasTex::getFBO");
		if(id != debug_thread_id_tex_created_in_) {
			cout << "this is notable! at least if this does not crash!," << endl << 
		          "It means that FBO and textures can be created in different threads" << 
		          endl;
		}
		glBindFramebuffer(GL_FRAMEBUFFER, 0);//TODO: do we really need this?
	}
	return fbo_->get();
}

int TexAtlasTex::countPatches() {
	int width = tex_->getWidth() / tile_size_;
	return width * width - free_slots_.size();
}

TexAtlasPatch::TexAtlasPatch(shared_ptr<TexAtlasTex> &tex, cv::Rect2i &pos, 
                             int index) 
		: tex_(tex),
		  pos_(pos),
		  index_in_atlas_tex_(index) {
}

TexAtlasPatch::~TexAtlasPatch() {
	tex_->freeTexSlot_(index_in_atlas_tex_);
}

shared_ptr<gfx::GpuTex2D> TexAtlasPatch::getTex() {
	return tex_->tex_;
}

cv::Rect2i TexAtlasPatch::getPosition() {
	return pos_;
}

GLuint TexAtlasPatch::getFBO() {
	return tex_->getFBO();
}

void TexAtlasPatch::setViewport() {
	setViewport(cv::Size2i(pos_.width, pos_.height));
}

void TexAtlasPatch::setViewport(cv::Size2i size) {
	glViewport(pos_.x, pos_.y, size.width, size.height);
}

bool TexAtlasPatch::downloadData(void *data, cv::Rect2i rect) {
	if(rect.x + rect.width  >= tex_->tex_->getWidth() || 
	   rect.y + rect.height >= tex_->tex_->getHeight()) {
		return false;
	} else {
		tex_->tex_->downloadData(data, rect.x, rect.y, rect.width, rect.height);
		return true;
	}
}

bool TexAtlasPatch::uploadData(void *data, cv::Rect2i rect) {
	int width = tex_->tex_->getWidth();
	int height = tex_->tex_->getHeight();

	if(rect.x + rect.width  >= tex_->tex_->getWidth() || 
	   rect.y + rect.height >= tex_->tex_->getHeight()) {
		return false;
	} else {
		tex_->tex_->uploadData(data, rect.x, rect.y, rect.width, rect.height);
		return true;
	}
}

void TexAtlasPatch::showImage(string text) {
	showImage(text, cv::Size2i(pos_.width, pos_.height));
}

void TexAtlasPatch::showImage(string text, cv::Size2i size) {
	tex_->showImage(text, cv::Rect2i(pos_.x, pos_.y, size.width, size.height));
}

GpuTextureInfo TexAtlasPatch::genTexInfo(cv::Size2i size, 
                                         int tex_coord_starting_index) {
	int width = tex_->tex_->getWidth();
	int height = tex_->tex_->getHeight();

	GpuTextureInfo tex_info;
	tex_info.tex_coord_start_ind = tex_coord_starting_index;
	tex_info.gl_tex_pointer = tex_->tex_->getGlHandle();
	tex_info.pos   = Vector2f(float(pos_.x) / float(width), 
	                          float(pos_.y) / float(height));
	tex_info.size  = Vector2f(float(size.width) / float(width), 
	                          float(size.height) / float(height));
	tex_info._size = Vector2f(1.0f / tex_info.size[0], 
	                          1.0f / tex_info.size[1]);
	return tex_info;
}

GpuTextureInfo TexAtlasPatch::genTexInfo(int tex_coord_starting_index) {
	cv::Rect2i roi = getRect();
	return genTexInfo(roi.size(), tex_coord_starting_index);
}

cudaSurfaceObject_t TexAtlasPatch::getCudaSurfaceObject() {
	return getTex()->getCudaSurfaceObject();
}

cudaTextureObject_t TexAtlasPatch::getCudaTextureObject() {
	return getTex()->getCudaTextureObject();
}
uint64_t TexAtlasPatch::getGlHandle() {
	return getTex()->getGlHandle();
}