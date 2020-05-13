//
// Created by Osip on 2020-05-02.
//

#include "Utilities.h"
#include "DoubleComparison.h"

#include <array>
#include <algorithm>

namespace Geometry {

    std::ostream& operator<<(std::ostream& os, const Primitive& primitive) {
        char begin, end;
        switch (primitive.type_) {
            case 'V':
                begin = '{';
                end = '}';
                break;
            case 'P':
                begin = '(';
                end = ')';
                break;
            default:
                throw std::runtime_error("Error! Bad primitive type!");
        }

        os << begin << primitive.x_
           << "; " << primitive.y_
           << "; " << primitive.z_ << end;

        return os;
    }

    std::istream& operator>>(std::istream& is, Primitive& primitive) {
        // Point format:  (x_; y_; z_)
        // Vector format: {x_; y_; z_}
        is.ignore(1);
        is >> primitive.x_;
        is.ignore(1);
        is >> primitive.y_;
        is.ignore(1);
        is >> primitive.z_;

        return is;
    }

    Vector Normalize(const Vector& v) {
        Vector temp(v);
        temp.Normalize();

        return temp;
    }

    bool NullVector(const Vector& v) {
        return Equal(v.x_, 0.) && Equal(v.y_, 0.) && Equal(v.z_, 0.);
    }

    bool Collinear(const Vector& lhs, const Vector& rhs) { // Assumming both lhs and rhs are non-null vectors
        using std::array;
        using Pair = std::pair<double, double>;

        if (NullVector(lhs) || NullVector(rhs)) {
            throw std::invalid_argument("Null vector");
        }

        array<Pair, 3> coefs = {
                std::make_pair(lhs.x_, rhs.x_),
                std::make_pair(lhs.y_, rhs.y_),
                std::make_pair(lhs.z_, rhs.z_)
        };

        auto removeNull = std::remove_if(coefs.begin(), coefs.end(),
                                         [](const Pair& p) {
                                             return Equal(p.first, 0.) && Equal(p.second, 0.);
                                         });

        const double ratio = coefs[0].first / coefs[0].second;
        for (auto it = coefs.begin() + 1; it != removeNull; ++it) {
            if (!Equal(it->first / it->second, ratio)) {
                return false;
            }
        }

        return true;
    }

    double ScalarProduct(const Vector& lhs, const Vector& rhs) {
        return lhs.x_ * rhs.x_ +
               lhs.y_ * rhs.y_ +
               lhs.z_ * rhs.z_;
    }

    std::ostream& operator<<(std::ostream& os, const Plane& surface) {
        const auto coefs = surface.getCoefs();

        os << coefs[0] << "X ";

        if (coefs[1] > 0) {
            os << "+ " << coefs[1] << "Y ";
        } else {
            os << "- " << fabs(coefs[1]) << "Y ";
        }

        if (coefs[2] > 0) {
            os << "+ " << coefs[2] << "Z ";
        } else {
            os << "- " << fabs(coefs[2]) << "Z ";
        }

        if (coefs[3] > 0) {
            os << "+ " << coefs[3] << " = 0;";
        } else {
            os << "- " << fabs(coefs[3]) << " = 0;";
        }

        return os;
    }

}
