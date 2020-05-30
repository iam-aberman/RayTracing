//
// Created by Osip Chin on 27.05.2020.
//

#include "lighting.h"
#include "utilities.h"

RGB CalculateColor(Geometry3D::Vector& normal, Geometry3D::Vector& to_light,
                   const RGB& color) {
    normal.Normalize();
    to_light.Normalize();

    double d = ScalarProduct(normal, to_light);

    if (d > 1) {
        throw std::runtime_error("Vectors not normalized!");
    } else if (d < 0) {
        return {0, 0, 0};
    } else {
        int red = static_cast<int>(d * color.rgb_[0]),
                green = static_cast<int>(d * color.rgb_[1]),
                blue = static_cast<int>(d * color.rgb_[2]);

        return {static_cast<unsigned char>(red),
                static_cast<unsigned char>(green),
                static_cast<unsigned char>(blue)};
    }
}

RGB GetColor(const std::shared_ptr<Geometry3D::Object>& o, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color) {
    using namespace Geometry3D;

    RGB res;
    char type = o->getType();

    // Dynamic dispatching
    try {
        if (type == 'S') {
            const Sphere &s = dynamic_cast<Sphere &>(*o);
            res = GetColor(s, light, point, color);
        } else if (type == 'B') {
            const Box &b = dynamic_cast<Box &>(*o);
            res = GetColor(b, light, point, color);
        } else if (type == 'T') {
            const Tetrahedron &t = dynamic_cast<Tetrahedron &>(*o);
            res = GetColor(t, light, point, color);
        } else {
            throw std::invalid_argument("Unknown object type");
        }
    } catch (std::bad_cast&) {}

    return res;
}

RGB GetColor(const Geometry3D::Sphere& s, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color) {
    using namespace Geometry3D;

    Vector normal = point - s.getCentre();
    Vector to_light =  light - point;
    /* normal.Normalize();
    to_light.Normalize();

    double d = ScalarProduct(normal, to_light);

    if (d > 1) {
        throw std::runtime_error("Vectors not normalized!");
    } else if (d < 0) {
        return {0, 0, 0};
    } else {
         int red = static_cast<int>(d * color.rgb_[0]),
             green = static_cast<int>(d * color.rgb_[1]),
             blue = static_cast<int>(d * color.rgb_[2]);

         return {static_cast<unsigned char>(red),
                 static_cast<unsigned char>(green),
                 static_cast<unsigned char>(blue)};
    } */
    return CalculateColor(normal, to_light, color);
}

RGB GetColor(const Geometry3D::Box& b, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color) {
    using namespace Geometry3D;
    Geometry3D::Vector normal;

    const auto& minVertex = b.getMinVertex();
    const auto& maxVertex = b.getMaxVertex();

    if (Equal(point.z_, maxVertex.z_)) {
        normal = Vector(0, 0, 1);
    } else if (Equal(point.x_, maxVertex.x_)) {
        normal = Vector(1, 0, 0);
    } else if (Equal(point.y_, maxVertex.y_)) {
        normal = Vector(0, 1, 0);
    } else if (Equal(point.z_, minVertex.z_)) {
        normal = Vector(0, 0, -1);
    } else if (Equal(point.x_, minVertex.x_)) {
        normal = Vector(-1, 0, 0);
    } else if (Equal(point.y_, minVertex.y_)) {
        normal = Vector(0, -1, 0);
    }

    Vector to_light =  light - point;

    return CalculateColor(normal, to_light, color);
}

RGB GetColor(const Geometry3D::Tetrahedron& t, const Geometry3D::Point& light,
             const Geometry3D::Point& point, const RGB& color) {
    using namespace Geometry3D;
    using Edge = std::array<Point, 3>;

    auto vertices = t.getVertices();
    std::vector<Edge> Edges = {
            {vertices[0], vertices[1], vertices[2]},
            {vertices[0], vertices[1], vertices[3]},
            {vertices[0], vertices[2], vertices[3]},
            {vertices[1], vertices[2], vertices[3]}
    };

    size_t i;
    Vector normal;
    for (i = 0; i < 4; ++i) {
        Vector v1(Edges[i][0], Edges[i][1]),
               v2(Edges[i][0], Edges[i][2]),
               v3(Edges[i][0], point);

        Vector cross = CrossProduct(v2, v1);
        double d = ScalarProduct(cross, v3);
        if (Equal(d, 0.)) {
            normal = cross;
            break;
        }
    }

    if (i == 4) {
        throw std::runtime_error("Something has gone wrong");
    }

    Ray tmpRay(point, normal);
    size_t nIntersections = 0;
    for (const auto& edge : Edges) {
        if (RayTriangleIntersection(tmpRay, edge).has_value()) {
            ++nIntersections;
        }
    }

    if (nIntersections > 1) {
        normal = -normal;
    }
    Vector to_light =  light - point;

    return CalculateColor(normal, to_light, color);
}

