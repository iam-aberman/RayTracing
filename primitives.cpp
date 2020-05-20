//
// Created by Osip on 2020-04-29.
//

#include "primitives.h"

#include <array>
#include <cmath>

namespace Geometry3D {

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

    Vector::Vector(const std::array<double, 4>& coefs) : Primitive('V') {
        x_ = coefs[0];
        y_ = coefs[1];
        z_ = coefs[2];
    }

    Vector::Vector(const Point& other) : Primitive(other)
    {
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

    Vector Normalize(const Vector& v) {
        Vector temp(v);
        temp.Normalize();

        return temp;
    }

    Vector operator-(const Vector& v) {
        return Vector(-v.x_, -v.y_, -v.z_);
    }

    Ray::Ray(const Point& origin, const Vector& direction) :
            origin_(origin),
            direction_(Normalize(direction))
    {
    }

   /*  const Point& Ray::getOrigin() const {
        return origin_;
    }

    const Vector& Ray::getDirection() const {
        return direction_;
    }
*/
}