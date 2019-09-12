#ifndef FILE_INTERMEDIATE_DEPTH_MODEL_H
#define FILE_INTERMEDIATE_DEPTH_MODEL_H

#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <GL/glew.h>
#include <shader.h>

/*
 * TODO: create an intermediate model of stuff that isn't put into the reconstruction
 * already. this should be used for rendering. and mainly camera tracking
 */

class IntermediateMap {
public:

	IntermediateMap(int width,int height, Eigen::Vector4f d_intrinsics);
	~IntermediateMap();

	//maybe also add color
	void setDepthMap(cv::Mat depth16,Eigen::Matrix4f pose_at_capture);

	cv::Mat renderDepthMap(Eigen::Matrix4f pose);

private:

	/* ideally this should be done in opengl... but we are lazy and therefore are doing it in
	 * cuda
	gfx::GLSLProgram program;

	GLuint vertexBuffer;
	GLuint indexBuffer;
	GLuint VAO;


	GLuint FBO;
	GLuint depthRenderTex;
	GLuint depthBuf;
	*/
	int width_;
	int height_;

	// both buffers reside on gpu
	uint16_t *in_depth_;
	uint32_t *out_depth_32_;
	uint16_t *out_depth_16_;

	Eigen::Vector4f d_intrinsics_;

	Eigen::Matrix4f cam_pose_at_capture_;

};

#endif
