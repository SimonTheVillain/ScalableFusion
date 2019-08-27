
#ifndef FILE_GLV_GLUtils
#define FILE_GLV_GLUtils





#include <stdio.h>
#include <string>


#include <GL/glew.h>
//#include <GL/gl.h>


namespace gfx
{
    namespace GLUtils{
        int checkForOpenGLError(std::string msg);

        int checkOpenGLFramebufferStatus(std::string msg,GLenum tgtFBO=GL_FRAMEBUFFER);

        void dumpGLInfo(bool dumpExtensions = false);



        /*void debugOpenGLCallback( GLenum source, GLenum type, GLuint id,
         GLenum severity, GLsizei length, const GLchar * msg, const void * param );
         */
    }
}

#endif /* defined(__GLV__GLUtils__) */
