cmake_minimum_required(VERSION 3.14)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp primitives.cpp primitives.h plane.cpp plane.h DoubleComparison.h utilities.cpp utilities.h scene_objects.cpp scene_objects.h)
