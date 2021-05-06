#include "py_api.h"

void create_shader(MainWindow* window, char* path)
{
	window->create_shader(path);
}


MainWindow* create_window(const unsigned int WIDTH,
		const unsigned int HEIGHT,
		const char* title)
{
	return new MainWindow(WIDTH, HEIGHT, title);
}

void close(MainWindow* window)
{
	glfwSetWindowShouldClose(window->window, true);
}

bool is_close(MainWindow* window)
{
	return glfwWindowShouldClose(window->window);
}

void render(MainWindow* window)
{
	window->render();
}

void terminate()
{
	glfwTerminate();
}

Model* create_model(char* name)
{
	return new Model(name);
}

void add_permanent_to_window(MainWindow* window, Model* model)
{
	window->add_permanent_drawables(model);
}

void add_instantaneous_to_window(MainWindow* window, Model* model)
{
	window->add_instantaneous_drawables(model);
}

void translate_model(Model* model, float x, float y, float z)
{
	model->translate(glm::vec3(x, y, z));
}

void rotate_model(Model* model, float angle, float x, float y, float z)
{
	model->rotate(angle, glm::vec3(x, y, z));
}

void scale_model(Model* model, float x, float y, float z)
{
	model->scale(glm::vec3(x, y, z));
}

float get_fps(MainWindow* window)
{
	return window->FPS;
}

void set_fps(MainWindow* window, float fps)
{
	window->FPS_limit = fps;
}

Camera* get_camera(MainWindow* window)
{
	return window->camera;
}

void set_camera_pos(Camera* camera, float x, float y, float z)
{
	camera->Position = glm::vec3(x, y, z);
	camera->updateCameraVectors();
}

float* get_camera_pos(Camera* camera)
{
	return glm::value_ptr(camera->Position);
}


bool is_visible(MainWindow* window)
{
	int visible = glfwGetWindowAttrib(window->window, GLFW_VISIBLE);
	return visible;
}

void hide_window(MainWindow* window)
{
	glfwHideWindow(window->window);
}

void show_window(MainWindow* window)
{
	glfwShowWindow(window->window);
}
