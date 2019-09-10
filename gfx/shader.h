#ifndef FILE_SHADER_H
#define FILE_SHADER_H

#include <string>
#include <map>
#include <stdexcept>

#include <stdio.h>

#include <eigen3/Eigen/Eigen>

#include "glUtils.h"

using namespace std;

//TODO: use Eigen instead
namespace gfx {

class GLSLProgramException : public std::runtime_error {
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

	void compileShader(string fileName);
	void compileShader(const char *fileName);
	void compileShader(const char *fileName, GLSLShader::GLSLShaderType type);
	void compileShader(const string &source, GLSLShader::GLSLShaderType type,
	                   const char *fileName = NULL );

	void link();
	void validate();
	void use();

	int getHandle();
	bool isLinked();

	void bindAttribLocation( GLuint location, const char * name);
	void bindFragDataLocation( GLuint location, const char * name );

	void setUniform(const char *name, float x, float y, float z);
	void setUniform(const char *name, const Eigen::Vector2f &v);
	void setUniform(const char *name, const Eigen::Vector3f &v);
	void setUniform(const char *name, const Eigen::Vector4f &v);
	void setUniform(const char *name, const Eigen::Matrix3f &m);
	void setUniform(const char *name, const Eigen::Matrix4f &m);
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

	const char* getTypeString(GLenum type);

private:

	int  handle_;
	bool linked_;
	map<string, int> uniform_locations_;

	bool fileExists_(const string &fileName);
	string getExtension_(const char *fileName);

	// Make these private in order to make the object non-copyable
	GLSLProgram(const GLSLProgram &other) { }
	GLSLProgram& operator=(const GLSLProgram &other) {return *this;}

};
} // namespace gfx

#endif
