cmake_minimum_required(VERSION 3.14)
project(RayTracing)

include_directories(/usr/X11/include)
set(CMAKE_CXX_FLAGS "-Dcimg_display=0")
set(CMAKE_CXX_STANDARD 20)

add_executable(RayTracing main.cpp primitives.cpp primitives.h plane.cpp plane.h DoubleComparison.h utilities.cpp utilities.h scene_objects.cpp scene_objects.h profiler.h)
