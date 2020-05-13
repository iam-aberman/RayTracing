//
// Created by Osip on 2020-04-29.
//

#include "Primitives.h"
#include "DoubleComparison.h"

#include <array>
#include <cmath>

namespace Geometry {

    Primitive::Primitive(char type) : x_(0.), y_(0.), z_(0.), type_(type)
    {
    }

    Primitive::Primitive(double x, double y, double z, char type) : x_(x), y_(y), z_(z), type_(type)
    {
    }

    Primitive& Primitive::operator=(const Primitive& other) {
        if (type_ != other.type_) {
            throw std::invalid_argument("Error! Different Primitive types!");
        }

        x_ = other.x_;
        y_ = other.y_;
        z_ = other.z_;

        return *this;
    }

    Point::Point() : Primitive('P')
    {
    }

    Point::Point(double x, double y, double z) : Primitive(x, y, z, 'P')
    {
    }

    Vector::Vector() : Primitive('V')
    {
    }

    Vector::Vector(double x, double y, double z) : Primitive(x, y, z, 'V')
    {
    }

    Vector::Vector(const Point& begin, const Point& end) : Primitive('V') {
        x_ = end.x_ - begin.x_;
        y_ = end.y_ - begin.y_;
        z_ = end.z_ - begin.z_;
    }

    void Vector::Normalize() {
        const double curLen = getAbs();

        x_ /= curLen;
        y_ /= curLen;
        z_ /= curLen;
    }

    double Vector::getAbs() const {
        return sqrt(x_ * x_ + y_ * y_ + z_ * z_);
    }

    Ray::Ray(const Point& start, const Vector& direction) :
    start_(start),
    direction_(direction)
    {
    }

}