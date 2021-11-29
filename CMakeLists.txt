cmake_minimum_required(VERSION 2.8.12)
project(scan2cad_rasterizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3)

add_subdirectory(pybind11)
include_directories(eigen)

pybind11_add_module(scan2cad_rasterizer 
    src/main.cpp 
    src/Rasterizer.h
    src/Rasterizer.cpp 
    src/Grid.h 
    src/Triangle.h
)
