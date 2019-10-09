#ifndef FILE_SHADER_H
#define FILE_SHADER_H

#include <string>
#include <map>

#include <eigen3/Eigen/Eigen>

#include "glUtils.h"

using namespace std;
using namespace Eigen;

//TODO: use Eigen instead
namespace gfx {

class GLSLProgramException : public runtime_error {
public:

	GLSLProgramException(const string &msg) : runtime_error(msg) { }

};

namespace GLSLShader {

	enum GLSLShaderType {
		VERTEX          = GL_VERTEX_SHADER,
		FRAGMENT        = GL_FRAGMENT_SHADER,
		GEOMETRY        = GL_GEOMETRY_SHADER,
		TESS_CONTROL    = GL_TESS_CONTROL_SHADER,
		TESS_EVALUATION = GL_TESS_EVALUATION_SHADER,
	#ifndef __APPLE__
		COMPUTE         = GL_COMPUTE_SHADER
	#endif
	};

} // namespace GLSLShader

class GLSLProgram {
public:

	GLSLProgram();

	~GLSLProgram();

	void compileShader(string file_name);
	void compileShader(const char *file_name);
	void compileShader(const char *file_name, GLSLShader::GLSLShaderType type);
	void compileShader(const string &source, GLSLShader::GLSLShaderType type,
	                   const char *file_name = NULL );

	void link();
	void validate();
	void use();

	int getHandle();
	bool isLinked();

	void bindAttribLocation( GLuint location, const char * name);
	void bindFragDataLocation( GLuint location, const char * name );

	void setUniform(const char *name, float x, float y, float z);
	void setUniform(const char *name, const Vector2f &v);
	void setUniform(const char *name, const Vector3f &v);
	void setUniform(const char *name, const Vector4f &v);
	void setUniform(const char *name, const Matrix3f &m);
	void setUniform(const char *name, const Matrix4f &m);
	void setUniform(const char *name, float val);
	void setUniform(const char *name, int val);
	void setUniform(const char *name, bool val);
	void setUniform(const char *name, GLuint val);

	GLint getAttribLocation(string attrib_name);
	GLint getUniformLocation(string uniform_name);
	GLint getUniformLocation(const char *name);

	void printActiveUniforms();
	void printActiveUniformBlocks();
	void printActiveAttribs();

	const char *getTypeString(GLenum type);

private:

	int  handle_;
	bool linked_;
	map<string, int> uniform_locations_;

	bool fileExists_(const string &file_name);
	string getExtension_(const char *file_name);

	// Make these private in order to make the object non-copyable
	GLSLProgram(const GLSLProgram &other) { }
	GLSLProgram& operator=(const GLSLProgram &other) {return *this;}

};

} // namespace gfx

#endif // FILE_SHADER_H
