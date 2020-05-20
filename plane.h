//
// Created by Osip on 2020-04-29.
//

#ifndef RAYTRACING_PLANE_H
#define RAYTRACING_PLANE_H

#include "primitives.h"

#include <array>

namespace Geometry3D {

    class Plane {
    public:

        Plane() = default;
        Plane(const Point& f, const Point& s, const Point& t);

        void UpdateCoefs();

        std::array<double, 4> getCoefs() const;
        Vector getNormal() const;


        double A_, B_, C_, D_;
    private:

        // Ax + By + Cz + D = 0
        // double A_, B_, C_, D_;

    };

}


#endif //RAYTRACING_PLANE_H
