cmake_minimum_required(VERSION 3.14)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp Primitives.cpp Primitives.h Plane.cpp Plane.h DoubleComparison.h Utilities.cpp Utilities.h StageObjects.cpp StageObjects.h)
