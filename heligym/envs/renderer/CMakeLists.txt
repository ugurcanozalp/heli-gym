# taken from https://github.com/Shot511/OpenGLSampleCmake/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(Heligym)

# Add .lib files
link_directories(${CMAKE_SOURCE_DIR}/libs)

if(NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "Release")
endif()

SET(CMAKE_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE})


# Add source files
file(GLOB_RECURSE SOURCE_FILES 
	${CMAKE_SOURCE_DIR}/src/*.c
	${CMAKE_SOURCE_DIR}/src/*.cpp)
	
# Add header files
file(GLOB_RECURSE HEADER_FILES 
	${CMAKE_SOURCE_DIR}/src/*.h
	${CMAKE_SOURCE_DIR}/src/*.hpp)

# Configure assets header file
include_directories(${CMAKE_BINARY_DIR}/src)
	
# Define the executable/shared object or dynamic library
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
#add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

if (WIN32)	
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
	endif()
endif()

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/python
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/python
    LIBRARY_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/python
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/python
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}/python
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/python
	EXECUTABLE_OUTPUT_PATH   ${CMAKE_CURRENT_SOURCE_DIR}/python
)

# We need a CMAKE_DIR with some code to find external dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

#######################################
# LOOK for the packages that we need! #
#######################################

# GLFW
find_package(GLFW3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")

# GLM
find_package(GLM REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")

# OpenGL
find_package(OpenGL REQUIRED)

# ASSIMP
find_package(ASSIMP REQUIRED)
message(STATUS "Found ASSIMP in ${ASSIMP_INCLUDE_DIR}")

# STB
find_package(STB REQUIRED)
message(STATUS "Found STB in ${STB_INCLUDE_DIR}")

# GLAD
add_library(GLAD SHARED "${CMAKE_SOURCE_DIR}/src/glad.c")
set_target_properties(GLAD
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
	EXECUTABLE_OUTPUT_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/libs"
)

# IMGUI
# Set where the ImGui files are stored
set(IMGUI_PATH  "${CMAKE_CURRENT_SOURCE_DIR}/include/imgui")
    
# Compile as static library 
file(GLOB IMGUI_SOURCES ${IMGUI_PATH}/*.cpp) 
if (WIN32)
	add_library(imgui ${IMGUI_SOURCES})
else()
	add_library(imgui SHARED ${IMGUI_SOURCES})
endif()
target_include_directories(imgui PUBLIC ${IMGUI_PATH})

set_target_properties(imgui
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/libs"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/libs"
	EXECUTABLE_OUTPUT_PATH   "${CMAKE_CURRENT_SOURCE_DIR}/libs"
)

# Add/Link resources 
file(COPY ${data} DESTINATION resources)

# Put all libraries into a variable
if (WIN32)
	set(LIBS glfw3 GLAD opengl32 assimp imgui )
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(LIBS gdi32)
	endif()
else()
	set(LIBS glfw GL GLAD assimp imgui)
	#set(LIBS X11 Xrandr Xinerama Xxf86vm Xcursor GL dl pthread)
endif()



# Define the include DIRs
include_directories(
	"${CMAKE_SOURCE_DIR}/src"
	"${CMAKE_SOURCE_DIR}/include"
)

# Define the link libraries
target_link_libraries(${PROJECT_NAME} ${LIBS})


# Create virtual folders to make it look nicer in VS
if(MSVC_IDE)
	# Macro to preserve source files hierarchy in the IDE
	macro(GroupSources curdir)
		file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir} ${PROJECT_SOURCE_DIR}/${curdir}/*)

		foreach(child ${children})
			if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
				GroupSources(${curdir}/${child})
			else()
				string(REPLACE "/" "\\" groupname ${curdir})
				string(REPLACE "src" "Sources" groupname ${groupname})
				source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			endif()
		endforeach()
	endmacro()

	# Run macro
	GroupSources(src)
endif()
