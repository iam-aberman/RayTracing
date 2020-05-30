//
// Created by Osip Chin on 27.05.2020.
//

#ifndef RAYTRACING_LIGHTING_H
#define RAYTRACING_LIGHTING_H

#include "primitives.h"
#include "scene_objects.h"

#include <memory>

struct RGB {

    unsigned char rgb_[3] = {255, 255, 255};

};

RGB CalculateColor(Geometry3D::Vector& normal, Geometry3D::Vector& to_light,
                   const RGB& color);
RGB GetColor(const std::shared_ptr<Geometry3D::Object>& o, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color);
RGB GetColor(const Geometry3D::Sphere& s, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color);
RGB GetColor(const Geometry3D::Box& b, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color);
RGB GetColor(const Geometry3D::Tetrahedron& t, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color);

#endif //RAYTRACING_LIGHTING_H
