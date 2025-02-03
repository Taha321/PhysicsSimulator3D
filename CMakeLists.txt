cmake_minimum_required(VERSION 3.15)
project(RenderingFramework)
set (CMAKE_CXX_STANDARD 14)


set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)

# Set output directories for libraries and executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $<1:${CMAKE_BINARY_DIR}/output/bin>)

# Add the rendering framework 3D project directory
add_subdirectory(lib/RenderingFramework3D)

# Include project headers
include_directories(include)
include_directories(lib/RenderingFramework3D/include)

link_directories(lib/RenderingFramework3D/lib/glfw/x86_64)
link_directories(${VULKAN_DIR}/Lib)
link_directories(${VULKAN_DIR}/x86_64/lib)

# Source files (excluding test_scene.cpp)
file(GLOB_RECURSE SRC_FILES
    source/*.cpp
)

#build base test scene
add_executable(physim3d ${SRC_FILES})

#link test scene with rendering framework library
target_link_libraries(physim3d rfw3d)