cmake_minimum_required(VERSION 3.16)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

include_directories(/usr/X11/include)
set(CMAKE_CXX_FLAGS "-Dcimg_display=0")

add_executable(RayTracing
        double_comparison.h
        main.cpp
        plane.cpp
        plane.h
        primitives.cpp
        primitives.h
        profiler.h
        scene_objects.cpp
        scene_objects.h
        utilities.cpp
        utilities.h screen.cpp screen.h lighting.cpp lighting.h)
