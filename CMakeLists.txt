# define the cmake version
cmake_minimum_required(VERSION 2.8.0)

# define the project name
project(ode4solver)

# set the version
set(CMAKE_CXX_STANDARD 17)

# output setting
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# include the include folder
include_directories(${PROJECT_SOURCE_DIR}/include)

# include the eigen header folder
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/eigen3)

# add src folder
add_subdirectory(src)