//
// Created by Osip on 2020-04-29.
//

#include "plane.h"
#include "utilities.h"

namespace Geometry3D {

    Plane::Plane(const Point& f, const Point& s, const Point& t) {
        Vector v1(f, s), v2(f, t);
        if (Collinear(v1, v2)) {
            throw std::runtime_error("Two collinear vectors");
        }

        A_ = v1.y_ * v2.z_ - v2.y_ * v1.z_;
        B_ = v1.z_ * v2.x_ - v2.z_ * v1.x_;
        C_ = v1.x_ * v2.y_ - v2.x_ * v1.y_;
        D_ = f.x_ * (v2.y_ * v1.z_ - v1.y_ * v2.z_) +
             f.y_ * (v2.z_ * v1.x_ - v1.z_ * v2.x_) +
             f.z_ * (v2.x_ * v1.y_ - v1.x_ * v2.y_);

        UpdateCoefs();
    }

    void Plane::UpdateCoefs() {
        if (A_ < 0) {
            A_ *= -1;
            B_ *= -1;
            C_ *= -1;
            D_ *= -1;
        }
    }

    std::array<double, 4> Plane::getCoefs() const {
        return {A_, B_, C_, D_};
    }

    Vector Plane::getNormal() const {
        return Normalize({A_, B_, C_});
    }

}