//
// Created by Osip on 2020-05-02.
//

#ifndef RAYTRACING_UTILITIES_H
#define RAYTRACING_UTILITIES_H

#include "Primitives.h"
#include "Plane.h"

namespace Geometry {

    std::ostream& operator<<(std::ostream& os, const Primitive& primitive);
    std::ostream& operator<<(std::ostream& os, const Plane& surface);

    std::istream& operator>>(std::istream& is, Primitive& primitive);

    bool Collinear(const Vector& lhs, const Vector& rhs);

    Vector Normalize(const Vector& v);
    double ScalarProduct(const Vector& lhs, const Vector& rhs);

}

#endif //RAYTRACING_UTILITIES_H
