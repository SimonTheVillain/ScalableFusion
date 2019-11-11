// Example program
#include <iostream>
#include <string>
#include <fstream>

#include <Eigen/Eigen>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
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
	   glfwGetKey(window,GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
		center_camera = true;
	}
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	(void) window;
	(void) xoffset;
	float scale = 0.05f;
	dist_from_ball *= (1.0f + static_cast<float>(yoffset) * scale);
}

bool next_step = false;
void key_callback(GLFWwindow *window, int key, int scancode, int action, 
                  int mods) {
	if(key == GLFW_KEY_SPACE && action == GLFW_PRESS) {
		next_step = true;
	}
}

struct Intrinsics {
	int width, height;
	float cx, cy;
	float fx, fy;
	float k1, k2, k3;
	float p1, p2;

	void read(string line) {
		sscanf(line.c_str(),"%d , %d , %f , %f , %f , %f , %f , %f ,%f ,%f ,%f",
		       &width, &height, &cx, &cy, &fx, &fy, &k1, &k2, &k3, &p1, &p2);
	}
};

ostream &operator<<(ostream &out, const Intrinsics &i) {
	out << "cx " << i.cx << " cy " << i.cy << " fx " << i.fx << " fy " << i.fy ;
	out << " k1,k2,k3 " << i.k1 << ", " << i.k2 << ", " << i.k3;
	out << " p1,p2 " << i.p1 << ", " << i.p2;
	return out;
}

int main() {
	string path =
			"/home/simon/datasets/structure_core/single_shots";

	string extrinsics_file = path + "/extrinsics.txt";

	string intrinsics_file = path + "/intrinsics.txt";

	Matrix4f imu_from_depth;
	Matrix4f imu_from_color;
	fstream in;
	in.open(extrinsics_file);
	string line;
	getline(in, line);
	for(int i = 0; i < 16; i++) {
		in >> imu_from_depth(i);
	}
	cout << imu_from_depth << endl;

	getline(in, line);
	getline(in, line);
	for(int i = 0; i < 16; i++) {
		in >> imu_from_color(i);
	}
	cout << imu_from_color << endl;
	in.close();

	//INTRINSICS:
	Intrinsics intrinsics_color;
	Intrinsics intrinsics_depth;
	Intrinsics intrinsics_ir;

	in.open(intrinsics_file);
	getline(in, line);
	intrinsics_color.read(line);

	getline(in, line);
	intrinsics_ir.read(line);

	getline(in, line);
	intrinsics_depth.read(line);

	cout << intrinsics_ir << endl;
	cout << intrinsics_color << endl;
	cout << intrinsics_depth << endl;

	int image_nr = 1;

	cv::Mat depth;
	cv::Mat ir;
	cv::Mat rgb;
	string im_path = path + "/depth/" + to_string(image_nr) + ".png";
	depth = cv::imread(im_path);

	im_path = path + "/rgb/" + to_string(image_nr) + ".png";
	rgb = cv::imread(im_path);

	im_path = path + "/ir/" + to_string(image_nr) + ".png";
	ir = cv::imread(im_path);

	cv::imshow("rgb", rgb);
	cv::imshow("ir", ir);
	cv::imshow("depth", depth * 10);
	cv::waitKey();

	for(size_t i = 0; i < depth.rows; i++) {
		for(size_t j = 0; j < depth.cols; j++) {
			//TODO: all the projection bullshit, transformation and what not
		}
	}

	//TODO: use the simple
	return 0;

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
		trans_from_arcball(2, 3) = -dist_from_ball;
		Matrix4f mat = proj * trans_from_arcball * arcball.getView() * translation;
		glfwSwapBuffers(window);
		glfwPollEvents();

		int width, height;
		glfwGetWindowSize(window, &width, &height);
		arcball.setFramebufferData(width, height);

		if(next_step) {
			next_step = false;
		}
	}
	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}
