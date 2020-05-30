//
// Created by Osip on 2020-04-29.
//

#ifndef RAYTRACING_PLANE_H
#define RAYTRACING_PLANE_H

#include "primitives.h"
#include "double_comparison.h"

#include <array>

namespace Geometry3D {

    class Plane {
    public:

        Plane() = default;
        Plane(const Point& f, const Point& s, const Point& t);

        void UpdateCoefs() { // Intended to be inline
            if (A_ < 0) {
                A_ *= -1;
                B_ *= -1;
                C_ *= -1;
                D_ *= -1;
            }
        }

        std::array<double, 4> getCoefs() const { // Intended to be inline
            return {A_, B_, C_, D_};
        }

        Vector getNormal() const { // Intended to be inline
            return Normalize({A_, B_, C_});
        }

        bool IsPointOnPlane(const Point& p) const { // Intended to be inline
            return Equal((A_ * p.x_) + (B_ * p.y_) + (C_ * p.z_) + D_, 0.);
        }

    private:

        // Ax + By + Cz + D = 0
        double A_, B_, C_, D_;

    };

}


#endif //RAYTRACING_PLANE_H
