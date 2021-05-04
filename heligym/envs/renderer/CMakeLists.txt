# taken from https://github.com/Shot511/OpenGLSampleCmake/blob/master/CMakeLists.txt

cmake_minimum_required(VERSION 3.10)
project(Heligym)

# Add .lib files
link_directories(${CMAKE_SOURCE_DIR}/libs)

# set (source "${CMAKE_SOURCE_DIR}/shaders")
# set (destination "${CMAKE_CURRENT_BINARY_DIR}/shaders")
# add_custom_command(
#     TARGET ${PROJECT_NAME} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E create_symlink ${source} ${destination} 
#     DEPENDS ${destination}
#     COMMENT "symbolic link resources folder from ${source} => ${destination}"
# )


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

set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
	EXECUTABLE_OUTPUT_PATH   "${PROJECT_SOURCE_DIR}/python"
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
add_library(GLAD "${CMAKE_SOURCE_DIR}/src/glad.c")
set_target_properties(GLAD
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/python"
	EXECUTABLE_OUTPUT_PATH   "${PROJECT_SOURCE_DIR}/python"
)


# Add/Link resources 
file(COPY ${data} DESTINATION resources)

# Put all libraries into a variable
if (WIN32)
	set(LIBS glfw3 GLAD opengl32 assimp )
	if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(LIBS gdi32)
	endif()
else()
	set(LIBS glfw GLAD assimp )
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