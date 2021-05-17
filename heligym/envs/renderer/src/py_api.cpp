#include "py_api.h"
#include <cstdint>


Window* create_window(const unsigned int WIDTH,
		const unsigned int HEIGHT,
		const char* title)
{
	return new Window(WIDTH, HEIGHT, title);
}


void close(Window* window)
{
	glfwSetWindowShouldClose(window->window, true);
}


bool is_close(Window* window)
{
	return glfwWindowShouldClose(window->window);
}


void render(Window* window)
{
	window->render();
}


void terminate()
{
	glfwTerminate();
}

Model* create_model(char* model_path, char* vertex_shader_file_path, char* fragment_shader_file_path)
{
	return new Model(model_path, vertex_shader_file_path, fragment_shader_file_path);
}


void add_permanent_to_window(Window* window, Model* model)
{
	window->add_permanent_drawables(model);
}


void add_instantaneous_to_window(Window* window, Model* model)
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


float get_fps(Window* window)
{
	return window->FPS;
}


void set_fps(Window* window, float fps)
{
	window->FPS_limit = fps;
	std::chrono::nanoseconds dt{static_cast<long int>( 1000000000.0f/window->FPS_limit)};
	window->dt = dt;
}


Camera* get_camera(Window* window)
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



bool is_visible(Window* window)
{
	int visible = glfwGetWindowAttrib(window->window, GLFW_VISIBLE);
	return visible;
}


void hide_window(Window* window)
{
	glfwHideWindow(window->window);
}


void show_window(Window* window)
{
	glfwShowWindow(window->window);
}


void add_guiOBS(Window* window, int size, char** _str, float* _val)
{
	for (int i = 0; i < size; i++)
	{
		window->add_item_to_guiText(&window->guiOBS, _str[i], &_val[i] );
	}
}


void set_guiOBS(Window* window, float* _val)
{
	window->set_guiOBS(_val);
}


void rotate_MR(Model* model, float phi, float theta, float psi)
{
	model->mainrotor = glm::vec3(phi, theta, psi);
}


void rotate_TR(Model* model, float phi, float theta, float psi)
{
	model->tailrotor = glm::vec3(phi, theta, psi);
}