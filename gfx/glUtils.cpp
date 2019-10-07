//
//  GLUtils.cpp
//  vision
//
//  Created by Simon Schreiberhuber on 15.04.15.
//  Copyright (c) 2015 Simon Schreiberhuber. All rights reserved.
//

#include "glUtils.h"

#include <cstdio>
#include <string>
#include <iostream>

#include <assert.h>

using namespace std;

namespace gfx {
namespace GLUtils{

// Returns 1 if an OpenGL error occurred, 0 otherwise.
int checkForOpenGLError(string msg) {
	int    ret_code = 0;	
	GLenum gl_error = glGetError();
	while(gl_error != GL_NO_ERROR) {
		string error_msg;
		switch(gl_error)	{
			case GL_INVALID_ENUM:
				error_msg = "GL_INVALID_ENUM";
				break;
			case GL_INVALID_VALUE:
				error_msg = "GL_INVALID_VALUE";
				break;
			case GL_INVALID_OPERATION:
				error_msg = "GL_INVALID_OPERATION";
				break;
			case GL_INVALID_FRAMEBUFFER_OPERATION:
				error_msg = "GL_INVALID_FRAMEBUFFER_OPERATION";
				break;
			case GL_OUT_OF_MEMORY:
				error_msg = "GL_OUT_OF_MEMORY";
				break;
			default:
				error_msg = "Unknown OpenGL error";
		}

		cout << msg << "Error: " << error_msg << endl;
		ret_code = 1;
		gl_error = glGetError();

		#ifdef DEBUG
		assert(0);
		#endif
	}
	return ret_code;
}

int checkOpenGLFramebufferStatus(string msg, GLenum tgt_FBO) {
	GLenum gl_error = glCheckFramebufferStatus(tgt_FBO);
	if(gl_error) {
		string error_msg;
		switch(gl_error) {
			case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_DRAW_BUFFER";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_READ_BUFFER";
				break;
			case GL_FRAMEBUFFER_UNSUPPORTED:
				error_msg = "GL_FRAMEBUFFER_UNSUPPORTED";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_MULTISAMPLE";
				break;
			case GL_FRAMEBUFFER_UNDEFINED:
				error_msg = "GL_FRAMEBUFFER_UNDEFINED";
				break;
			case GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS:
				error_msg = "GL_FRAMEBUFFER_INCOMPLETE_LAYER_TARGETS";
				break;
			case GL_FRAMEBUFFER_COMPLETE:
				error_msg = "GL_FRAMEBUFFER_COMPLETE";
				return 0;
				break;
			default:
				break;
		}

		cout << msg << "Error: " << error_msg << endl;

		#ifdef DEBUG
		assert(0);
		#endif
	}
	return 0;
}

void dumpGLInfo(bool dump_extensions) {
	const GLubyte *renderer    = glGetString(GL_RENDERER);
	const GLubyte *vendor      = glGetString(GL_VENDOR);
	const GLubyte *version     = glGetString(GL_VERSION);
	const GLubyte *glslVersion = glGetString(GL_SHADING_LANGUAGE_VERSION);

	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);

	printf("-------------------------------------------------------------\n");
	printf("GL Vendor    : %s\n", vendor);
	printf("GL Renderer  : %s\n", renderer);
	printf("GL Version   : %s\n", version);
	printf("GL Version   : %d.%d\n", major, minor);
	printf("GLSL Version : %s\n", glslVersion);
	printf("-------------------------------------------------------------\n");

	if(dump_extensions) {
		GLint n_extensions;
		glGetIntegerv(GL_NUM_EXTENSIONS, &n_extensions);
		for(int i = 0; i < n_extensions; i++) {
			cout << glGetStringi(GL_EXTENSIONS, i) << endl;
		}
	}
}

} // namespace GLUtils
} // namespace gfx
