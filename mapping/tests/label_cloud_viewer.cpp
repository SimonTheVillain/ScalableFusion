// Example program
#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Eigen>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <gfx/camera.h>
#include <utils/arcball.h>
#include <rendering/renderable_model.h>
#include <graph/deformation_graph.h>

#define MAX_LABELS 21

using namespace std;
using namespace Eigen;

static Arcball arcball;


//TODO: get coordinates
static float dist_from_ball = 3.0f;
static Vector3f trans;

static bool capture_left = false;
static bool capture_right = false;
static double xpos_old = 0, ypos_old = 0;
static void cursor_position_callback(GLFWwindow *window, double xpos, 
                                     double ypos) {
	(void) window;

	float dx = static_cast<float>(xpos_old - xpos);
	float dy = static_cast<float>(ypos_old - ypos);
	if(capture_left) {
		float speed = 0.05f;
		Vector4f trans4(-dx * speed, dy * speed, 0, 1);
		trans4 = arcball.getView().inverse() * trans4;
		trans += trans4.block<3, 1>(0, 0);
	}
	if(capture_right) {
		//do the arcball thingy with the right bouse button
		arcball.drag(xpos, ypos);
	}
	xpos_old = xpos;
	ypos_old = ypos;
}

static bool read_out_surface_info = false;
static bool center_camera = false;
void mouse_button_callback(GLFWwindow *window, int button, int action, 
                           int mods) {
	(void) mods;
	if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
		capture_left = true;
	}
	if(button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
		capture_left = false;
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
		capture_right = true;
		arcball.clickBegin(static_cast<float>(xpos_old),
		                   static_cast<float>(ypos_old));
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
		capture_right=false;
	}
	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
	   glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
		read_out_surface_info = true;
	}

	if(button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS &&
	   glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		center_camera = true;
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	(void) window;
	(void) xoffset;

	float scale = 0.05f;
	dist_from_ball *= (1.0f + static_cast<float>(yoffset) * scale);
}

bool next_step=false;
void key_callback(GLFWwindow *window, int key, int scancode, int action, 
                  int mods) {
	if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
		next_step = true;
	}
}

int main() {
	string label_colors_file =
			"/home/simon/datasets/EdithCloudBla/label_color_map.txt";

	string points_file =
			"/home/simon/datasets/EdithCloudBla/coordsConf.txt";

	struct Point {
		Vector4f pos;
		int label[MAX_LABELS];
		float confidence[MAX_LABELS];
	};

	vector<Point> points;
	vector<Vector4f> colors;

	fstream file;
	file.open(label_colors_file);

	if(!file.is_open()) {
		assert(0);
	}

	string line;
	while (getline(file, line)) {
		istringstream iss(line);
		int r, g, b;
		char separator;
		iss >> r >> separator >> g >> separator >> b;
		cout << r << " " << g << " " << b << endl;
		colors.push_back(Vector4f(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f));
	}
	cout << "yeah" << endl;
	file.close();
	file.open(points_file);
	if(!file.is_open()) {
		assert(0);
	}

	while(getline(file, line)) {
		Point p;
		istringstream iss(line);
		float x, y, z;
		iss >> x >> y >> z;
		cout << x << " " << y << " " << z << " ";
		p.pos = Vector4f(x, y, z, 1.0f);
		for(int i = 0; i < MAX_LABELS; i++) {
			iss >> p.label[i] >> p.confidence[i];
			cout << p.label[i] << " " << p.confidence[i] << " ";
		}
		cout << endl;
		points.push_back(p);
	}

	int node_count;
	int edge_count;
	file >> node_count;
	cout << node_count << endl;
	file >> edge_count;

	vector<Vector4f> nodes(node_count);
	vector<Vector3f> nodes3(nodes.size());
	for(size_t i = 0; i < node_count; i++) {
		Vector4f p;
		file >> p[0] >> p[1] >> p[2];
		p[3] = 1;
		nodes[i] = p;
		nodes3[i] = Vector3f(p[0], p[1], p[2]);
		nodes3[i] *= 0.1f;
	}
	vector<unsigned int> edges(edge_count * 2);
	for(size_t i = 0; i < edge_count; i++) {
		int edge1, edge2;
		file >> edge1 >> edge2;
		edges[i * 2] = edge1;
		edges[i * 2 + 1] = edge2;
	}

	file.close();
	cout << "Graph is loaded! Hopefully!" << endl;

	DeformationGraph deformation_graph;

	deformation_graph.indices = edges;
	deformation_graph.nodes = nodes3;

	PinConstraint cons;
	cons.node = 0;
	cons.pos = nodes3[0];
	deformation_graph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it


	cons.node = 1;
	cons.pos = nodes3[1];
	deformation_graph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it


	cons.node = 2;
	cons.pos = nodes3[2];
	deformation_graph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it

	cons.node = 3;
	cons.pos = nodes3[3];
	deformation_graph.pin_constraints.push_back(cons);//only one constraint should shift everything towards it

	for(int i = 0; i < 100; i++) {
		cons.node = i;
		cons.pos = nodes3[i];
		deformation_graph.pin_constraints.push_back(cons);
	}

	for(int i = deformation_graph.nodes.size() - 100; 
	    i < deformation_graph.nodes.size(); i++) {
		cons.node = i;
		cons.pos = nodes3[i];
		cons.pos[1] += 0.5;
		deformation_graph.pin_constraints.push_back(cons);
	}

	deformation_graph.generateNeighbourList();
	deformation_graph.initializeTransforms();
	for(int i = 0; i < 1; i++) {
		deformation_graph.generateResiduals();
		deformation_graph.generateJacobian();
		deformation_graph.gaussNewtonStep();
	}

	if (!glfwInit())
		exit(EXIT_FAILURE);


	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	GLFWwindow *window;
	window = glfwCreateWindow(1280, 1024, "graph", nullptr, nullptr);

	//set the callback functions for the different inputs
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetKeyCallback(window, key_callback);

	glfwMakeContextCurrent(window);

	glewExperimental = GL_TRUE;
	glewInit();
	glGetError();

	//******************SETTING UP SHADER***************************


	GLuint ssbo_colors;
	glGenBuffers(1, &ssbo_colors);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_colors);
	glBufferData(GL_SHADER_STORAGE_BUFFER, 
	             sizeof(Vector4f) * colors.size(), 
	             &colors[0], GL_STATIC_DRAW);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, ssbo_colors);
	glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0); // unbind

	GLuint ssbo_points;

	//**************************************************************

	RenderVerySimpleModel simple_model;
	simple_model.setMesh(nodes, edges);

	RenderVerySimpleModel simple_model2;
	simple_model2.color = Vector4f(1, 1, 0, 1);
	vector<Vector4f> nodes2 = nodes;

	for(int i = 0; i < nodes.size(); i++) {
		nodes2.push_back(nodes[i]);
		nodes2[i](0) += deformation_graph.G_t[i](0);
		nodes2[i](1) += deformation_graph.G_t[i](1);
		nodes2[i](2) += deformation_graph.G_t[i](2);
	}

	simple_model2.setMesh(nodes2,edges);

	while(!glfwWindowShouldClose(window)) {

		glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);

		//render:

		Matrix4f proj = Camera::projection(static_cast<float>(M_PI) * 0.3f,
		                                   800.0f / 600.0f);
		Matrix4f translation = Matrix4f::Identity();
		translation.block<3, 1>(0, 3) = trans;
		Matrix4f trans_from_arcball = Matrix4f::Identity();
		trans_from_arcball(2,3) = -dist_from_ball;
		Matrix4f mat = proj * trans_from_arcball * arcball.getView() * translation;

		simple_model.render(mat);
		simple_model2.render(mat);

		glfwSwapBuffers(window);
		glfwPollEvents();

		int width, height;
		glfwGetWindowSize(window, &width, &height);
		arcball.setFramebufferData(width, height);

		if(next_step) {
			next_step = false;
			deformation_graph.generateResiduals();
			deformation_graph.generateJacobian();
			deformation_graph.gaussNewtonStep();

			vector<Vector4f> nodes2;
			for(int i = 0; i < nodes.size(); i++) {
				nodes2.push_back(nodes[i]);
				nodes2[i](0) += deformation_graph.G_t[i](0);
				nodes2[i](1) += deformation_graph.G_t[i](1);
				nodes2[i](2) += deformation_graph.G_t[i](2);
			}
			simple_model2.setMesh(nodes2, edges);
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
