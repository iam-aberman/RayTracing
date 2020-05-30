//
// Created by Osip on 2020-04-29.
//

#ifndef RAYTRACING_GEOMETRY_H
#define RAYTRACING_GEOMETRY_H

#include <iostream>
#include <array>
#include <cmath>

namespace Geometry3D {

    struct Primitive {

        double x_;
        double y_;
        double z_;

        const char type_;

        Primitive() = delete;
        explicit Primitive(char type = 'A');
        Primitive(double x, double y, double z, char type = 'A');

        virtual ~Primitive() = default;

        Primitive(const Primitive& other) = default;
        Primitive& operator=(const Primitive& other);

        std::array<double, 3> getNums() const { // Intended to be inline
            return {x_, y_, z_};
        }

    };

    struct Point : public Primitive {

        Point();
        Point(double x, double y, double z);

        Point(const Point& other) = default;
        using Primitive::operator=;

    };

    struct Vector : public Primitive {

        Vector();
        Vector(double x, double y, double z);
        Vector(const Point& begin, const Point& end);
        Vector(const std::array<double, 4>& coefs);

        // This is for casting purposes
        Vector(const Point& other);

        Vector(const Vector& other) = default;
        using Primitive::operator=;

        void Normalize() { // Intended to be inline
            const double curLen = getAbs();

            x_ /= curLen;
            y_ /= curLen;
            z_ /= curLen;
        }

        double getAbs() const {
            return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
        }

    };

    Vector Normalize(const Vector& v);
    Vector operator-(const Vector& v);

    struct Ray {

        Ray(const Point& origin, const Vector& direction);

        const Point  origin_;
        const Vector direction_;

    };


}

#endif //RAYTRACING_GEOMETRY_H
