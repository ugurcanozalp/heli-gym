# **heli-gym Rendering**
To render heli-gym, OpenGL is used. OpenGL written in C++ and connect with Python with shared libraries DLL (for Windows) and so (for Linux) as a part of API. By using Python's `ctypes` module, API can be used by loading shared libraries.

OpenGL structure based on this [Tutorial](https://learnopengl.com). The tutorial is very effective and suggested for who interested to learn OpenGL.

If any C++ files changed somehow (adding new methods etc.) API should be re-compiled. To build and compile the API, [CMake](https://cmake.org) is used. 

---
---

## **Build CMake for Windows OS**
In Windows, [MSVC](https://visualstudio.microsoft.com/vs/features/cplusplus/) is suggested compiler to build DLL file. To use MSVC for building API in Windows, there is two way. 

---

_**First**_ way is using CMake GUI. In GUI firstly set source code folder which is `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer`. After that set build folder as `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/build`. If there is no `build` folder, GUI pop up an dialog box to warn you and accept it. After setting the folder, click the `Configure` button. Select the MSVC version and press `Finish`. If there is no error (which written in RED color (warnings are also in RED))which means `Configuring done` message appear at the end of gui, click the `Generate` button. Click the `Open Project` button to open MSVC solution. Lastly, compile the MSVC solution and make sure that `config` in MSVC is `Release`.

---

_**Second**_ way is using cmd. Firstly call the 
```bash
cd ...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer
```
command. Create the `build` folder and change the directory to folder, call the 
```bash
mkdir build && cd build
```
command. Then, to build (like `Configure` and `Generate` in GUI), call the 
```bash
cmake ..
```
command. To compile in `Release` config, call the 
```bash
cmake --build . --config Release
```
command. 

These two way creates the `Heligym.dll` file which is shared libraries for Python and C++ linkage. The file should be in `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/python` folder. Even it setted to output to the `../python` folder, make sure that your last compiled DLL file should be in the folder.

---
---

## **Build CMake for Linux**
In Linux, [GCC](https://gcc.gnu.org) is suggested compiler to build so file. To use GCC for building API in Linux, there is two way. 

---

_**First**_ way is using CMake GUI. In GUI firstly set source code folder which is `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer`. After that set build folder as `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/build`. If there is no `build` folder, GUI pop up an dialog box to warn you and accept it. After setting the folder, click the `Configure` button. Select the `Unix Makefiles` as generator and click the `Finish`. If there is no error (which written in RED color (warnings are also in RED)) which means `Configuring done` message appear at the end of gui, click the `Generate` button. Open the terminal, call the 

```bash
cd ...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/build
```
command to be in the build folder. Then to compile, call the 
```bash
make
```
command. Build type is assigned as `Release`. Therefore just `make` commant will work. 

---

_**Second**_ way is using terminal. Firstly call the 
```bash
cd ...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer
```
command. Create the `build` folder and change the directory to folder, call the 
```bash
mkdir build && cd build
```
command. Then, to build (like `Configure` and `Generate` in GUI), call the 
```bash
cmake ..  
``` 
or 
```bash
cmake -G "Unix Makefiles" #for the Unix Makefiles generator
```
command. Build type is assigned as `Release`. Yet, if you want to make sure, call the 
```bash
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
```
command. To compile, call the 
```bash
make
```
command. 

These two way creates the `libHeligym.so` file which is shared libraries for Python and C++ linkage. The file should be in `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/python` folder. Even it setted to output to the `../python` folder, make sure that your last compiled so file should be in the folder.

---
---

## **Dependent Libraries**
There is several libraries for OpenGL application. Some of them should be in shared libraries. Therefore, in some OS, these libraries should be re-compiled. These libraries are [Assimp](https://www.assimp.org) for asset importing and [GLFW](https://www.glfw.org) for creating windows, contexts and surfaces, receiving input and events.

In Windows OS, these libraries are not need to re-compile. The conflict occurs in Linux distros. To re-compile these libraries, CMake can be used.

---

### **Re-compile Assimp**
Heli-gym uses assimp 5.0.1. From [Assimp Download](https://github.com/assimp/assimp/releases/tag/v5.0.1), download the zip file (if you want you can use tar file also). Extract the file and open the terminal. Then call the,

```bash
cd ...path_to_assimp_5.0.1_/
```
command. Then, to create build folder and change directory in the folder call the

```bash
mkdir build && cd build
```
command. To build, call the 
```bash
cmake ..
```
command. To compile the assimp, call the 
```bash
make
```
command. `make` command creates `libassimp.so`, `libassimp.so.5` and `libassimp.so.5.0.1` in `...path_to_assimp_5.0.1_/build/bin` folder. These files should be copied to `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs` folder. It can be also done in terminal. First change directory to bin folder with
```bash
cd ...path_to_assimp_5.0.1_/build/bin
```
command. Then copy the files to `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs` with 
```bash
cp libassimp.so libassimp.so.5 libassimp.so.5.0.1 ...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs
```
command.

---

### **Re-compile GLFW**

Heli-gym uses GLFW 3.3.4. From [GLFW](https://www.glfw.org), download the zip file. Extract the file and open the terminal. Then call the,

```bash
cd ...path_to_glfw_3.3.4/
```
command. Then, to create build folder and change directory in the folder call the

```bash
mkdir build && cd build
```
command. To build for shared librares (for details look [Docs](https://www.glfw.org/docs/latest/compile.html#compile_deps_x11) ), call the 
```bash
cmake -DBUILD_SHARED_LIBS=ON .
```
command. To compile the glfw, call the 
```bash
make
```
command. `make` command creates `libglfw.so`, `libglfw.so.3` and `libglfw.so.3.3` in `...path_to_glfw_3.3.4/build/src` folder. These files should be copied to `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs` folder. It can be also done in terminal. First change directory to bin folder with
```bash
cd ...path_to_glfw_3.3.4/build/src
```
command. Then copy the files to `...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs` with 
```bash
cp libglfw.so libglfw.so.3 libglfw.so.3.3 ...path_to_heli-gym_folder.../heli-gym/heligym/envs/renderer/libs
```
command.