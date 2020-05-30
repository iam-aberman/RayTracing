//
// Created by Osip Chin on 20.05.2020.
//

#ifndef RAYTRACING_SCREEN_H
#define RAYTRACING_SCREEN_H

#include "scene_objects.h"
#include "lighting.h"

#include <string>
#include <vector>

struct Screen {

    Geometry3D::Point camera_, light_;
    double distance_, alpha_;
    Geometry3D::Vector normal_, yAxis_;
    size_t height_, width_;

    Screen() = default;

};

Screen LoadScreenParameters(const std::string& filename);

void Render(const Screen& screen, const std::vector<std::shared_ptr<Geometry3D::Object>>& objects,
            const std::vector<RGB>& colors);

void func(const Geometry3D::Point& camera, double distance, double alpha,
          Geometry3D::Vector& normal, Geometry3D::Vector& yAxis,
          size_t screenHeight, size_t screenWidth, const Geometry3D::Tetrahedron& s);

#endif //RAYTRACING_SCREEN_H
