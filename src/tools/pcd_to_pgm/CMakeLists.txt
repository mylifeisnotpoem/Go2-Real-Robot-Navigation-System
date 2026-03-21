cmake_minimum_required(VERSION 3.10)
project(pcd_to_pgm_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL REQUIRED COMPONENTS io filters)

add_executable(pcd_to_pgm_cpp pcd_to_pgm.cpp)
target_include_directories(pcd_to_pgm_cpp PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_to_pgm_cpp PRIVATE ${PCL_LIBRARIES})
target_compile_definitions(pcd_to_pgm_cpp PRIVATE ${PCL_DEFINITIONS})
