//
// Created by Osip on 2020-04-29.
//

#ifndef RAYTRACING_GEOMETRY_H
#define RAYTRACING_GEOMETRY_H

#include <iostream>

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

        Vector(const Vector& other) = default;
        using Primitive::operator=;

        void Normalize();
        double getAbs() const;

    };

    Vector Normalize(const Vector& v);

    class Ray {
    public:

        Ray(const Point& start, const Vector& direction);

        const Point& getOrigin() const;
        const Vector& getDirection() const;

    private:

        const Point  origin_;
        const Vector direction_;

    };


}

#endif //RAYTRACING_GEOMETRY_H
