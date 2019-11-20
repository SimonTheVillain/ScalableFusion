#ifndef FILE_DEBUG_RENDER_H
#define FILE_DEBUG_RENDER_H

#include <mutex>
#include <memory>

#include <gpu/shader.h>

class Meshlet;

class RenderDebugInfo {
public:

	struct ShowPatch {
		float r, g, b, a;
		Meshlet* patch;
	};

	RenderDebugInfo();

	void render(Eigen::Matrix4f proj, Eigen::Matrix4f cam_pose);
	void setIndexCount(int index, int count);

	void addPatch(Meshlet *patch, float r, float g, float b);
	void setPatch(Meshlet *patch);

	std::vector<ShowPatch> patches;

	GLuint vertex_buffer;
	GLuint triangle_buffer;
	GLuint info_buffer;
	GLuint tex_pos_buffer;

	bool force_dst_geom;

	Meshlet *rendered_patch;

	//also make the color configurable;

private:
	
	std::mutex mutex_;

	GLuint start_index_;
	GLuint count_;

	std::shared_ptr<gfx::GLSLProgram> shader_;

};

extern RenderDebugInfo *that_one_debug_rendering_thingy;

#endif // FILE_DEBUG_RENDER_H