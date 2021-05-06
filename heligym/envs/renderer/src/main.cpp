#include "gWindow.h"



// settings
const unsigned int SCR_WIDTH = 1200;
const unsigned int SCR_HEIGHT = 800;






int main()
{    
     // render loop
    // -----------
    MainWindow* window = new MainWindow(SCR_WIDTH, SCR_HEIGHT, "gGL");
    while (!glfwWindowShouldClose(window->window))
    {
        window->render();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}
