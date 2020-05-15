//
// Created by Osip on 2020-05-02.
//

#ifndef RAYTRACING_UTILITIES_H
#define RAYTRACING_UTILITIES_H

#include "primitives.h"
#include "plane.h"

namespace Geometry3D {

    std::ostream& operator<<(std::ostream& os, const Primitive& primitive);
    std::ostream& operator<<(std::ostream& os, const Plane& surface);

    std::istream& operator>>(std::istream& is, Primitive& primitive);

    Point operator+(const Point& point, const Vector& vector);
    Vector operator-(const Point& end, const Point& begin);
    Vector operator*(const Vector& v, double lambda);

    bool Collinear(const Vector& lhs, const Vector& rhs);

    inline double ScalarProduct(const Vector& lhs, const Vector& rhs) {
        return lhs.x_ * rhs.x_ +
               lhs.y_ * rhs.y_ +
               lhs.z_ * rhs.z_;
    }

}

#endif //RAYTRACING_UTILITIES_H
