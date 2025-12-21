cmake_minimum_required(VERSION 3.10)

project(rtt_planning)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17) 
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -DNDEBUG")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_executable(rtt_planning 
    src/Tree.cpp 
    src/RRT.cpp 
    src/test.cpp
)
target_link_libraries(rtt_planning Eigen3::Eigen)
target_include_directories(rtt_planning PRIVATE include)

find_package(Python 3.6 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(rtt_planning_lib
    src/Tree.cpp 
    src/RRT.cpp 
    src/bindings.cpp
)

target_compile_options(rtt_planning_lib PRIVATE -O3 -march=native -DNDEBUG)
target_include_directories(rtt_planning_lib PRIVATE include)
target_link_libraries(rtt_planning_lib PRIVATE Eigen3::Eigen)