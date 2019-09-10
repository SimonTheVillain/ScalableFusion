#ifndef FILE_GL_UTILS_H
#define FILE_GL_UTILS_H

#include <string>

#include <stdio.h>

#include <GL/glew.h>

using namespace std;

namespace gfx {
namespace GLUtils{

int checkForOpenGLError(string msg);

int checkOpenGLFramebufferStatus(string msg, GLenum tgt_FBO = GL_FRAMEBUFFER);

void dumpGLInfo(bool dump_extensions = false);

} // namespace GLUtils
} // namespace gfx

#endif
