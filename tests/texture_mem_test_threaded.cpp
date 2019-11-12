#include <iostream>
#include <vector>
#include <memory>
//mostly because we need to sleep for a longer time
#include <chrono>
#include <thread>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gfx/gpu_tex.h>
#include <gpu/thread_safe_FBO_VAO.h>
#include <gfx/garbage_collector.h>
#include <gfx/gl_utils.h>

using namespace std;

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

	GLFWwindow *other_context;
	other_context = glfwCreateWindow(640, 480, "HOPE U NO VISIBLE",
	                                 nullptr, invisible_window);

	vector<shared_ptr<gfx::GpuTex2D>> textures;
	int full_res = 1024;
	GLuint int_type = GL_RGBA32F;
	GLuint format = GL_RGBA;
	GLuint type = GL_FLOAT;
	ThreadSafeFBOStorage fbo_storage;
	vector<ThreadSafeFBO*> fbos;
	vector<uint64_t> handles;
	bool done = false;
	auto func = [&]() {
		if (!glfwInit())
			exit(EXIT_FAILURE);

		glfwMakeContextCurrent(other_context);
		glewExperimental = GL_TRUE;
		glewInit();
		glGetError();

		for(int i = 0; i < 400; i++) {
			auto tex = make_shared<gfx::GpuTex2D>(&garbage_collector, int_type,
			                                      format, type, full_res, full_res,
			                                      true, nullptr);
			glMakeTextureHandleResidentARB(tex->getGlHandle());
			handles.push_back(tex->getGlHandle());
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

			textures.push_back(tex);

			if(glIsTextureHandleResidentARB(tex->getGlHandle())) {

			} else {
				cout << "why should it not be resident?" << endl;
			}
		}
		fbo_storage.forceGarbageCollect();

		gfx::GLUtils::checkForOpenGLError(
				"is making texture resident a big mistake?");
		done = true;
	};

	thread t(func);
	while(!done) {
	}

	for(int i = 0; i < 400; i++) {
		auto tex = textures[i];
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
		gfx::GLUtils::checkForOpenGLError("is making texture resident a big mistake?");
	}

	t.join();

	this_thread::sleep_for(chrono::seconds(10));

	for(int i = 0; i < 400; i++) {
		auto tex = textures[i];
	}
	for(size_t i = 0; i < handles.size(); i++) {
		glMakeTextureHandleNonResidentARB(handles[i]);
		gfx::GLUtils::checkForOpenGLError("is making texture non resident a big mistake?");
	}
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

	for(size_t i = 0; i < handles.size(); i++) {
		gfx::GLUtils::checkForOpenGLError("is making texture non resident a big mistake?");
	}
	cout << "making the textures non-resident" << endl;
	this_thread::sleep_for(chrono::seconds(10));

	glfwDestroyWindow(invisible_window);
	glfwTerminate();
	cout << "OpenGLContext should be cleared" << endl;

	this_thread::sleep_for(chrono::seconds(10));

	cout << "It is over now" << endl;

	return 0;
}