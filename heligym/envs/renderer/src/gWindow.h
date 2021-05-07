#ifndef GWINDOW_H
#define GWINDOW_H

#include "camera.h"
#include "model.h"

#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <iostream>

struct guiText
{
    char* str;
    float* val;
    guiText() { str = 0; val = 0; }
    guiText(char* _str, float* _val) { str = _str; val = _val; }
};

class MainWindow
{
private:
    unsigned int SCR_WIDTH;
    unsigned int SCR_HEIGHT;
    float deltaTime = 0.0f;
    float lastFrame = 0.0f;
    float updateTime = 0.0f;
    float lastX = 0.0f;
    float xoffset = 0.0f;
    float lastY = 0.0f;
    float yoffset = 0.0f;
    bool firstMouse = true;
    bool focues = false;

    std::vector<Model*> permanent_drawables;
	std::vector<Model*> instantaneous_drawables;
    void draw();

public:
    GLFWwindow* window;
    Shader* ourShader;
    Camera* camera;

    float FPS = 1e-7;
    float FPS_limit = 50.0; 

    std::vector<guiText> guiOBS;

    MainWindow() {};

    MainWindow(const unsigned int SCR_WIDTH,
                const unsigned int SCR_HEIGHT,
                const char* title);

    void create_shader(std::string shader_file_path);
    
    void render();

    void renderGUI();

    void set_guiOBS(float* val);

    void add_permanent_drawables(Model* drawable);
	void add_instantaneous_drawables(Model* drawable);

    void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void mouse_callback(GLFWwindow* window, double xpos, double ypos);
    void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    void processInput(GLFWwindow* window);
    void window_focus_callback(GLFWwindow* window, int focused);

    void add_item_to_guiText(std::vector<guiText>* guiText, char* str, float* val);

    static void static_framebuffer_size_callback(GLFWwindow* window, int width, int height);
    static void static_mouse_callback(GLFWwindow* window, double xpos, double ypos);
    static void static_scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
    static void static_window_focus_callback(GLFWwindow* window, int focused);
        
};
#endif