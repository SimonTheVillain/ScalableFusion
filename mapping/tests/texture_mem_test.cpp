#include <iostream>
#include <vector>
#include <memory>
//mostly because we need to sleep for a longer time
#include <chrono>
#include <thread>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../../gfx/gpu_tex.h"
#include "../gpu/thread_safe_FBO_VAO.h"
#include "../../gfx/garbage_collector.h"
#include "../../gfx/gl_utils.h"

using namespace std;

/*
https://www.khronos.org/registry/OpenGL/extensions/ARB/ARB_bindless_texture.txt

(7) What happens if you try to delete a texture or sampler object with a
handle that is resident in another context?

RESOLVED:  Deleting the texture will remove the texture from the name
space and make all handles using the texture non-resident in the current
context.  However, texture or image handles for a deleted texture are
not deleted until the underlying texture or sampler object itself is
deleted.  That deletion won't happen until the object is not bound
anywhere and there are no handles using the object that are resident in
		any context.
*/
int main(int argc, const char *argv[]) {
	cout << "starting allocating tons of memory on a single thread" << endl;

	GarbageCollector garbage_collector;
	glfwInit();
	GLFWwindow *invisible_window;
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_VISIBLE, 0);
	invisible_window = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE", 
	                                    nullptr, nullptr);

	glfwMakeContextCurrent(invisible_window);
	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();

	vector<shared_ptr<gfx::GpuTex2D>> textures;
	int full_res = 1024;
	GLuint int_type = GL_RGBA32F;
	GLuint format = GL_RGBA;
	GLuint type = GL_FLOAT;
	ThreadSafeFBOStorage fbo_storage;
	vector<ThreadSafeFBO*> fbos;
	vector<uint64_t> handles;
	for(int i = 0; i < 400; i++) {
		auto tex = make_shared<gfx::GpuTex2D>(&garbage_collector, int_type,
		                                      format,type, full_res, full_res,
		                                      true, nullptr);
		auto fbo = fbo_storage.createGlObject();
		glBindFramebuffer(GL_FRAMEBUFFER, fbo->get());
		glBindTexture(GL_TEXTURE_2D, tex->getGlName());//not necessary
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
		                     tex->getGlName(), 0);
		GLenum draw_buffers[1] = {GL_COLOR_ATTACHMENT0};
		glDrawBuffers(1, draw_buffers);

		GLenum gl_err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if(gl_err != GL_FRAMEBUFFER_COMPLETE) {
			cout << "framebuffer incomplete" << endl;
		}
		fbos.push_back(fbo);

		//TODO: also get a fbo
		//TODO: also make them resident!!!!!!!
		uint64_t handle = tex->getGlHandle();
		glMakeTextureHandleResidentARB(handle);
		gfx::GLUtils::checkForOpenGLError(
				"is making texture resident a big mistake?");
		handles.push_back(handle);

		textures.push_back(tex);
	}

	this_thread::sleep_for(chrono::seconds(10));

	textures.clear();

	//destroy
	cout << "textures should be cleared" << endl;

	this_thread::sleep_for(chrono::seconds(10));
	for(auto fbo : fbos) {
		delete fbo;
	}
	fbos.clear();
	fbo_storage.garbageCollect();
	glFinish();
	cout << "fbos should be cleared" << endl;

	this_thread::sleep_for(chrono::seconds(10));

	for(int i = 0; i < handles.size(); i++) {
		glMakeTextureHandleNonResidentARB(handles[i]);
		gfx::GLUtils::checkForOpenGLError(
				"is making texture non resident a big mistake?");
	}
	cout << "texture handles made non-resident" << endl;

	this_thread::sleep_for(chrono::seconds(10));

	for(int i = 0; i < handles.size(); i++) {
		gfx::GLUtils::checkForOpenGLError(
				"is making texture resident again a big mistake?");
	}
	cout << "making textures resident again" << endl;
	this_thread::sleep_for(chrono::seconds(10));

	glfwDestroyWindow(invisible_window);
	glfwTerminate();
	cout << "OpenGLContext should be cleared" << endl;

	this_thread::sleep_for(chrono::seconds(10));

	cout << "It is over now" << endl;

	return 0;
}