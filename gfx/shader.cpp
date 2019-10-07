//
//  GLSLProgram.cpp
//  vision
//
//  Created by Simon Schreiberhuber on 15.04.15.
//  Copyright (c) 2015 Simon Schreiberhuber. All rights reserved.
//

#include "shader.h"

#include <fstream>
#include <sstream>
#include <iostream>

#include <sys/stat.h>

using namespace std;
using namespace gfx;

namespace GLSLShaderInfo {

struct shader_file_extension {
	const char *ext;
	GLSLShader::GLSLShaderType type;
};

struct shader_file_extension extensions[] = {
	{".vs",   GLSLShader::VERTEX},
	{".vsh",  GLSLShader::VERTEX},
	{".vert", GLSLShader::VERTEX},
	{".gs",   GLSLShader::GEOMETRY},
	{".gsh",  GLSLShader::GEOMETRY},
	{".geom", GLSLShader::GEOMETRY},
	{".tcs",  GLSLShader::TESS_CONTROL},
	{".tes",  GLSLShader::TESS_EVALUATION},
	{".fs",   GLSLShader::FRAGMENT},
	{".fsh",  GLSLShader::FRAGMENT},
	{".frag", GLSLShader::FRAGMENT},
#ifndef __APPLE__
	{".cs",   GLSLShader::COMPUTE},
	{".csh",  GLSLShader::COMPUTE}
#endif
};

} // namespace GLSLShaderInfo

GLSLProgram::GLSLProgram()
		: handle_(0), 
		  linked_(false) { 
}

GLSLProgram::~GLSLProgram() {
	if(handle_ == 0)
		return;

	// Query the number of attached shaders
	GLint num_shaders = 0;
	glGetProgramiv(handle_, GL_ATTACHED_SHADERS, &num_shaders);

	// Get the shader names
	GLuint *shader_names = new GLuint[num_shaders];
	glGetAttachedShaders(handle_, num_shaders, NULL, shader_names);

	// Delete the shaders
	for (int i = 0; i < num_shaders; i++)
		glDeleteShader(shader_names[i]);

	// Delete the program
	glDeleteProgram(handle_);

	delete[] shader_names;
}

void GLSLProgram::compileShader(string filename) {
	compileShader(filename.c_str());
}

void GLSLProgram::compileShader(const char *filename) {
	int num_exts = sizeof(GLSLShaderInfo::extensions) / 
	               sizeof(GLSLShaderInfo::shader_file_extension);

	// Check the file name's extension to determine the shader type
	string ext = getExtension_(filename);
	GLSLShader::GLSLShaderType type = GLSLShader::VERTEX;
	bool match_found = false;
	for(int i = 0; i < num_exts; i++) {
		if(ext == GLSLShaderInfo::extensions[i].ext) {
			match_found = true;
			type = GLSLShaderInfo::extensions[i].type;
			break;
		}
	}

	// If we didn't find a match, throw an exception
	if(!match_found) {
		string msg("Unrecognized extension: " + ext);
		throw GLSLProgramException(msg);
	}

	// Pass the discovered shader type along
	compileShader(filename, type);
}

string GLSLProgram::getExtension_(const char *name) {
	string name_str(name);
	auto loc = name_str.find_last_of('.');
	if(loc != string::npos) {
		return name_str.substr(loc, string::npos);
	} else {
		return "";
	}
}

void GLSLProgram::compileShader(const char *filename,
                                GLSLShader::GLSLShaderType type) {
	if(!fileExists_(filename)) {
		string message = string("Shader: ") + filename + " not found.";
		throw GLSLProgramException(message);
	}

	if(handle_ <= 0) {
		handle_ = glCreateProgram();
		if(handle_ == 0) {
			throw GLSLProgramException("Unable to create shader program.");
		}
	}

	ifstream in_file(filename, ios::in);
	if(!in_file) {
		string message = string("Unable to open: ") + filename;
		throw GLSLProgramException(message);
	}

	// Get file contents
	stringstream code;
	code << in_file.rdbuf();
	in_file.close();

	compileShader(code.str(), type, filename);
}

void GLSLProgram::compileShader(const string &source,
                                GLSLShader::GLSLShaderType type,
                                const char *filename) {
	if(handle_ <= 0) {
		handle_ = glCreateProgram();
		if(handle_ == 0) {
			throw GLSLProgramException("Unable to create shader program.");
		}
	}

	GLuint shader_handle = glCreateShader(type);

	const char *c_code = source.c_str();
	glShaderSource(shader_handle, 1, &c_code, NULL);

	// Compile the shader
	glCompileShader(shader_handle);

	// Check for errors
	int result;
	glGetShaderiv(shader_handle, GL_COMPILE_STATUS, &result);
	if(GL_FALSE == result) {
		// Compile failed, get log
		int length = 0;
		string log_string;
		glGetShaderiv(shader_handle, GL_INFO_LOG_LENGTH, &length);
		if(length > 0) {
			char *c_log = new char[length];
			int written = 0;
			glGetShaderInfoLog(shader_handle, length, &written, c_log);
			log_string = c_log;
			delete [] c_log;
		}
		string msg;
		if(filename) {
			msg = string(filename) + ": shader compliation failed\n";
		} else {
			msg = "Shader compilation failed.\n";
		}
		msg += log_string;

		throw GLSLProgramException(msg);

	} else {
		// Compile succeeded, attach shader
		glAttachShader(handle_, shader_handle);
	}
}

void GLSLProgram::link() {
	if(linked_) 
		return;

	if(handle_ <= 0)
		throw GLSLProgramException("Program has not been compiled.");

	glLinkProgram(handle_);

	int status = 0;
	glGetProgramiv( handle_, GL_LINK_STATUS, &status);
	if(GL_FALSE == status) {
		// Store log and return false
		int length = 0;
		string log_string;

		glGetProgramiv(handle_, GL_INFO_LOG_LENGTH, &length);

		if(length > 0) {
			char * c_log = new char[length];
			int written = 0;
			glGetProgramInfoLog(handle_, length, &written, c_log);
			log_string = c_log;
			delete [] c_log;
		}

		//assert(0);//this is a debug measure ... delete this at the right opportunity.
		//(somehow gdb fails when resolving this exception)
		throw GLSLProgramException(string("Program link failed:\n") + log_string);
	} else {
		uniform_locations_.clear();
		linked_ = true;
	}
}

void GLSLProgram::use() {
	if((handle_ <= 0) || (!linked_))
		throw GLSLProgramException("Shader has not been linked");
	glUseProgram(handle_);
}

int GLSLProgram::getHandle() {
	return handle_;
}

bool GLSLProgram::isLinked() {
	return linked_;
}

void GLSLProgram::bindAttribLocation(GLuint location, const char *name) {
	glBindAttribLocation(handle_, location, name);
}

void GLSLProgram::bindFragDataLocation(GLuint location, const char * name) {
	glBindFragDataLocation(handle_, location, name);
}

void GLSLProgram::setUniform(const char *name, float x, float y, float z) {
	GLint loc = getUniformLocation(name);
	glUniform3f(loc, x, y, z);
}

void GLSLProgram::setUniform(const char *name, const Eigen::Vector3f &v) {
	setUniform(name, v[0], v[2], v[2]);
}

void GLSLProgram::setUniform( const char *name, const Eigen::Vector4f &v) {
	GLint loc = getUniformLocation(name);
	glUniform4f(loc, v[0], v[1], v[2], v[3]);
}

void GLSLProgram::setUniform( const char *name, const Eigen::Vector2f &v) {
	GLint loc = getUniformLocation(name);
	glUniform2f(loc, v[0], v[1]);
}

void GLSLProgram::setUniform( const char *name, const Eigen::Matrix4f &m) {
	GLint loc = getUniformLocation(name);
	glUniformMatrix4fv(loc, 1, GL_FALSE, (float*) &m);
}

void GLSLProgram::setUniform(const char *name, const Eigen::Matrix3f &m) {
	GLint loc = getUniformLocation(name);
	glUniformMatrix3fv(loc, 1, GL_FALSE, (float*)&m);
}

void GLSLProgram::setUniform(const char *name, float val) {
	GLint loc = getUniformLocation(name);
	glUniform1f(loc, val);
}

void GLSLProgram::setUniform(const char *name, int val) {
	GLint loc = getUniformLocation(name);
	glUniform1i(loc, val);
}

void GLSLProgram::setUniform(const char *name, GLuint val) {
	GLint loc = getUniformLocation(name);
	glUniform1ui(loc, val);
}

GLint GLSLProgram::getAttribLocation(string attribute_name) {
	return glGetAttribLocation(handle_, attribute_name.c_str());
}

GLint GLSLProgram::getUniformLocation(string uniform_name) {
	return glGetUniformLocation(handle_, uniform_name.c_str());
}

void GLSLProgram::setUniform(const char *name, bool val) {
	int loc = getUniformLocation(name);
	glUniform1i(loc, val);
}

void GLSLProgram::printActiveUniforms() {
	printf("Not implemented yet!! Sorry!!!\n");
}

void GLSLProgram::printActiveUniformBlocks() {
	printf("Not implemented yet!! Sorry!!!\n");
}

void GLSLProgram::printActiveAttribs() {
	printf("Not implemented yet!! Sorry!!!\n");
}

const char* GLSLProgram::getTypeString(GLenum type) {
	// There are many more types than are covered here, but
	// these are the most common in these examples.
	switch(type) {
		case GL_FLOAT:
			return "float";
		case GL_FLOAT_VEC2:
			return "vec2";
		case GL_FLOAT_VEC3:
			return "vec3";
		case GL_FLOAT_VEC4:
			return "vec4";
		case GL_DOUBLE:
			return "double";
		case GL_INT:
			return "int";
		case GL_UNSIGNED_INT:
			return "unsigned int";
		case GL_BOOL:
			return "bool";
		case GL_FLOAT_MAT2:
			return "mat2";
		case GL_FLOAT_MAT3:
			return "mat3";
		case GL_FLOAT_MAT4:
			return "mat4";
		case GL_SAMPLER_2D:
			return "sampler2D";
		default:
			return "?";
	}
}

void GLSLProgram::validate() {
	if(!isLinked())
		throw GLSLProgramException("Program is not linked");

	GLint status;
	glValidateProgram(handle_);
	glGetProgramiv(handle_, GL_VALIDATE_STATUS, &status);

	if(GL_FALSE == status) {
		// Store log and return false
		int length = 0;
		string log_string;

		glGetProgramiv(handle_, GL_INFO_LOG_LENGTH, &length);

		if(length > 0) {
			char *c_log = new char[length];
			int written = 0;
			glGetProgramInfoLog(handle_, length, &written, c_log);
			log_string = c_log;
			delete [] c_log;
		}

		throw GLSLProgramException(string("Program failed to validate\n") + log_string);
	}
}

int GLSLProgram::getUniformLocation(const char *name) {
	map<string, int>::iterator pos;
	pos = uniform_locations_.find(name);

	if(pos == uniform_locations_.end())
		uniform_locations_[name] = glGetUniformLocation(handle_, name);

	return uniform_locations_[name];
}

bool GLSLProgram::fileExists_(const string &filename) {
	struct stat info;
	int ret = stat(filename.c_str(), &info);
	return 0 == ret;
}
