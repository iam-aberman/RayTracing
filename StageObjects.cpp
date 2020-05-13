//
// Created by Osip on 2020-05-02.
//

#include "StageObjects.h"

namespace Geometry {

    Sphere::Sphere(const Point& centre, double radius) :
    centre_(centre),
    radius_(radius)
    {
    }

    const Point& Sphere::getCentre() const {
        return centre_;
    }

    double Sphere::getRadius() const {
        return radius_;
    }

    Box::Box(const Point& minPoint, const Point& maxPoint) {
        // Bottom layer
        vertices_[0] = minPoint;
        vertices_[1] = Point(maxPoint.x_, minPoint.y_, minPoint.z_);
        vertices_[2] = Point(maxPoint.x_, maxPoint.y_, minPoint.z_);
        vertices_[3] = Point(minPoint.x_, maxPoint.y_, minPoint.z_);

        // Top layer
        vertices_[4] = Point(minPoint.x_, minPoint.y_, maxPoint.z_);
        vertices_[5] = Point(maxPoint.x_, minPoint.y_, maxPoint.z_);
        vertices_[6] = maxPoint;
        vertices_[7] = Point(minPoint.x_, maxPoint.y_, maxPoint.z_);

        // Surfaces
        surfaces_[0] = Plane(vertices_[0], vertices_[1], vertices_[2]);
        surfaces_[1] = Plane(vertices_[4], vertices_[5], vertices_[6]);
        surfaces_[2] = Plane(vertices_[0], vertices_[1], vertices_[4]);
        surfaces_[3] = Plane(vertices_[2], vertices_[3], vertices_[6]);
        surfaces_[4] = Plane(vertices_[0], vertices_[3], vertices_[4]);
        surfaces_[5] = Plane(vertices_[1], vertices_[2], vertices_[5]);
    }

    const std::array<Point, 8>& Box::getVertices() const {
        return vertices_;
    }

    const std::array<Plane, 6> &Box::getSurfaces() const {
        return surfaces_;
    }

    Tetrahedron::Tetrahedron(const Point& A, const Point& B,
                             const Point& C, const Point& D) :
                             vertices_({A, B, C, D}) {
        surfaces_[0] = Plane(vertices_[0], vertices_[1], vertices_[2]);
        surfaces_[1] = Plane(vertices_[1], vertices_[2], vertices_[3]);
        surfaces_[0] = Plane(vertices_[0], vertices_[2], vertices_[3]);
        surfaces_[0] = Plane(vertices_[0], vertices_[1], vertices_[3]);
    }

    const std::array<Point, 4>& Tetrahedron::getVertices() const {
        return vertices_;
    }

    const std::array<Plane, 4>& Tetrahedron::getSurfaces() const {
        return surfaces_;
    }

}
