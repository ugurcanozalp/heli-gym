#include "gWindow.h"


MainWindow::MainWindow(const unsigned int SCR_WIDTH,
                        const unsigned int SCR_HEIGHT,
                        const char* title)
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    //glfwSwapInterval(0);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);


    #ifdef __APPLE__
            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    // glfw window creation
    // --------------------
    this->SCR_WIDTH = SCR_WIDTH;
    this->SCR_HEIGHT = SCR_HEIGHT;
    this->window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, title, NULL, NULL);
    if (this->window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
    }


    glfwMakeContextCurrent(this->window);


    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);  

    // camera
    this->camera = new Camera(glm::vec3(5.415f, 0.2f, 30.0f),
        glm::vec3(0.0f, 1.0f, 0.0f));

    this->lastX = SCR_WIDTH / 2.0f;
    this->lastY = SCR_HEIGHT / 2.0f;


    glfwSetInputMode(this->window, GLFW_STICKY_MOUSE_BUTTONS, GLFW_TRUE);

    glfwSetWindowUserPointer(this->window, this);
    glfwSetFramebufferSizeCallback(this->window, MainWindow::static_framebuffer_size_callback);
    glfwSetCursorPosCallback(this->window, MainWindow::static_mouse_callback);
    glfwSetScrollCallback(this->window, MainWindow::static_scroll_callback);

    // Create gui 
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(this->window, true);
    ImGui_ImplOpenGL3_Init("#version 150");


    this->add_item_to_guiText(&this->guiOBS, "FPS : %3.f", &this->FPS);

}

void MainWindow::create_shader(std::string shader_file_path)
{
    // build and compile our shader program
    // ------------------------------------
    std::string vertex_loc = shader_file_path + "/vertex.vs";
    std::string frag_loc = shader_file_path + "/frag.fs";
    this->ourShader = new Shader(vertex_loc.c_str(), frag_loc.c_str());
    this->ourShader->use();
}

void MainWindow::render()
{
    if (!glfwWindowShouldClose(this->window))
    {
        // render
        // ------
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                

        // activate shader
        this->ourShader->use();
       
        // pass projection matrix to shader (note that in this case it could change every frame)
        glm::mat4 projection = glm::perspective(glm::radians(this->camera->Zoom), (float)this->SCR_WIDTH / (float)this->SCR_HEIGHT, 0.1f, 2000.0f);

        // camera/view transformation
        glm::mat4 view = this->camera->GetViewMatrix();
        glm::mat4 p_v = projection * view;
        this->ourShader->setMat4("projection_view", p_v);    
        
        // draw objects
        this->draw();     
    
        // render gui
        this->renderGUI();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        //glfwSwapBuffers(this->window);
        glFlush();
        glfwPollEvents();        

        // per-frame time logic
        // --------------------
        this->deltaTime = std::chrono::duration_cast<std::chrono::nanoseconds> (std::chrono::steady_clock::now() - this->lastFrame);

        
        //if (this->deltaTime >= 1.0/this->FPS_limit)
        auto diff =  (this->dt - this->deltaTime);
        //std::cout << " DIF " << diff.count() << std::endl;
        if (diff.count() > 0)
        {
            auto t1 = std::chrono::steady_clock::now();
            //std::this_thread::sleep_for(diff); 
            this->preciseSleep(diff.count()/1e9);
            auto t2 = std::chrono::steady_clock::now();
            auto int_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1);
            //std::cout << "WAIT " << " " << int_ms.count() << std::endl;
        } 

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        auto dt1 =  std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - this->lastFrame);
        this->FPS = 1.0f / (dt1.count() / 1e9);
        //std::cout << "FPS " << this->FPS << " DT " <<  (dt1.count() / 1e9) << std::endl;

        this->lastFrame = std::chrono::steady_clock::now();

    }
    else
    {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
}


void MainWindow::preciseSleep(double seconds) {

    static double estimate = 5e-3;
    static double mean = 5e-3;
    static double m2 = 0;
    static int64_t count = 1;

    while (seconds > estimate) {
        auto start = std::chrono::steady_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        auto end = std::chrono::steady_clock::now();

        double observed = (end - start).count() / 1e9;
        seconds -= observed;

        ++count;
        double delta = observed - mean;
        mean += delta / count;
        m2   += delta * (observed - mean);
        double stddev = sqrt(m2 / (count - 1));
        estimate = mean + stddev;
    }

    // spin lock
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start).count() / 1e9 < seconds);
}

void MainWindow::renderGUI()
{
    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImVec2 info_pos = ImVec2(30.0f, 30.0f);
    ImGui::SetNextWindowPos(info_pos);
    ImGui::SetNextWindowSize(ImVec2(250, 0));
    ImGui::Begin("Observations!");

    for (int i = 0; i < this->guiOBS.size(); i++)
    {
        ImGui::Text(this->guiOBS[i].str, *this->guiOBS[i].val);
    }

    ImGui::End();

    ImGui::Render();
}


// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void MainWindow::processInput(GLFWwindow* window)
{
        
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void MainWindow::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}


// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void MainWindow::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = (float)xpos;
        lastY = (float)ypos;
        firstMouse = false;
    }

    this->xoffset = (float)xpos - lastX;
    this->yoffset = lastY - (float)ypos; // reversed since y-coordinates go from bottom to top

    lastX = (float)xpos;
    lastY = (float)ypos;


    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        this->camera->ProcessMouseMovement(-this->xoffset, -this->yoffset);
    }
       
}


// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void MainWindow::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    this->camera->ProcessMouseScroll((float)yoffset);
}

// glfw : window focus
void MainWindow::window_focus_callback(GLFWwindow* window, int focus)
{
    if (focus)
    {
        std::cout << "FOCUSED" << std::endl;
        //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }
    else
    {
        std::cout << "NOT FOCUSED" << std::endl;
        //glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

}

void MainWindow::static_framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    MainWindow* temp_window = static_cast<MainWindow*>(glfwGetWindowUserPointer(window));
    temp_window->framebuffer_size_callback(window, width, height);
}
void MainWindow::static_mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    MainWindow* temp_window = static_cast<MainWindow*>(glfwGetWindowUserPointer(window));
    temp_window->mouse_callback(window, xpos, ypos);
}
void MainWindow::static_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    MainWindow* temp_window = static_cast<MainWindow*>(glfwGetWindowUserPointer(window));
    temp_window->scroll_callback(window, xoffset, yoffset);
}
void MainWindow::static_window_focus_callback(GLFWwindow* window, int focused)
{
    MainWindow* temp_window = static_cast<MainWindow*>(glfwGetWindowUserPointer(window));
    temp_window->window_focus_callback(window, focused);
}



void MainWindow::add_permanent_drawables(Model* drawable)
{
	this->permanent_drawables.push_back(drawable);
}

void MainWindow::add_instantaneous_drawables(Model* drawable)
{
	this->instantaneous_drawables.push_back(drawable);
}

void MainWindow::draw()
{
	for (int i = 0; i < this->permanent_drawables.size(); i++)
	{
		this->permanent_drawables[i]->draw(*this->ourShader);
	}

	for (int i = 0; i < this->instantaneous_drawables.size(); i++)
	{
		this->instantaneous_drawables[i]->draw(*this->ourShader);
	}
	this->instantaneous_drawables.clear();
}


void MainWindow::add_item_to_guiText(std::vector<guiText>* _guiText, const char* str, float* val)
{
    guiText* temp = new guiText((char*) str, val);
    _guiText->push_back(*temp);
    delete temp;
}



void MainWindow::set_guiOBS( float* val)
{
    for (int i = 0; i < this->guiOBS.size(); i++)
    {
        //std::cout << str[i] << std::endl;
        //this->guiOBS[i + 1].str = str[i];
        this->guiOBS[i + 1].val = &val[i] ;
    }
}
