//
// Created by Osip on 2020-05-02.
//

#include "scene_objects.h"
#include "utilities.h"

#include <algorithm>
#include <fstream>

namespace Geometry3D {

    std::optional<Point> RayPlaneIntersection(const Ray &ray, const Plane &plane) {
        const auto coefs = plane.getCoefs();
        const Vector plane_normal = Vector(coefs);
        const double D = coefs[3];

        if (Equal(ScalarProduct(plane_normal, ray.direction_), 0.)) {
            return std::nullopt;
        } else {
            double t = (-D - ScalarProduct(static_cast<Vector>(ray.origin_), plane_normal)) /
                       ScalarProduct(ray.direction_, plane_normal);

            if (t < 0) {
                return std::nullopt;
            } else {
                return ray.origin_ + ray.direction_ * t;
            }
        }
    }

    std::optional<double> RayTriangleIntersection(const Ray &ray,
                                                  const std::array<Point, 3> &vertices) {
        const Vector E1 = vertices[1] - vertices[0],
                E2 = vertices[2] - vertices[0],
                P = CrossProduct(ray.direction_, E2);

        const double determinant = ScalarProduct(E1, P);
        if (fabs(determinant) < std::numeric_limits<double>::epsilon()) {
            return std::nullopt;
        }

        double invertedDet = 1 / determinant;

        Vector v1 = ray.origin_ - vertices[0];
        double u = ScalarProduct(v1, P) * invertedDet;
        if (u < 0 || u > 1) {
            return std::nullopt;
        }

        Vector v_2 = CrossProduct(v1, E1);
        double v = ScalarProduct(ray.direction_, v_2) * invertedDet;
        if (v < 0 || u + v > 1) {
            return std::nullopt;
        }

        double t = ScalarProduct(E2, v_2) * invertedDet;
        if (t > 0 || Equal(t, 0.)) {
            return t;
        } else {
            return std::nullopt;
        }
    }

    Object::Object(char type) : type_(type) {
    }

    Sphere::Sphere(const Point &centre, double radius) :
            Object('S'),
            centre_(centre),
            radius_(radius) {
    }

    const Point &Sphere::getCentre() const {
        return centre_;
    }

    double Sphere::getRadius() const {
        return radius_;
    }

    std::optional<Point> Sphere::Intersect(const Ray &ray) const {
        Vector sphere_to_origin = ray.origin_ - centre_;

        double B = 2. * ScalarProduct(sphere_to_origin, ray.direction_);
        double C = ScalarProduct(sphere_to_origin, sphere_to_origin) - radius_ * radius_;
        double discriminant = B * B - 4 * C;

        if (discriminant < 0) {
            return std::nullopt;
        } else {
            const double discriminantRoot = sqrt(discriminant);
            double t_1 = (-B + discriminantRoot) / 2.;
            double t_2 = (-B - discriminantRoot) / 2.;

            double min_t = std::min(t_1, t_2);
            double max_t = std::max(t_1, t_2);

            double tRes = (min_t >= 0) ? min_t : max_t;
            if (tRes < 0) {
                return std::nullopt;
            } else {
                return ray.origin_ + (ray.direction_ * tRes);
            }
        }
    }

    double Sphere::DistanceToPoint(const Point &point) const {
        return sqrt(ScalarProduct(Vector(centre_, point), Vector(centre_, point)));
    }

    Box::Box(const Point &minPoint, const Point &maxPoint) : Object('B'),
                                                             minVertex_(minPoint),
                                                             maxVertex_(maxPoint) {
    }

    std::optional<Point> Box::Intersect(const Ray &ray) const {
        const auto ray_origin = ray.origin_.getNums();
        const auto ray_direction = ray.direction_.getNums();

        if (minVertex_.x_ <= ray_origin[0] && ray_origin[0] <= maxVertex_.x_ &&
            minVertex_.y_ <= ray_origin[1] && ray_origin[1] <= maxVertex_.y_ &&
            minVertex_.z_ <= ray_origin[2] && ray_origin[2] <= maxVertex_.z_) {
            throw std::runtime_error("Camera inside an object!");
        }

        std::array<double, 3> minVert = {minVertex_.x_, minVertex_.y_, minVertex_.z_};
        std::array<double, 3> maxVert = {maxVertex_.x_, maxVertex_.y_, maxVertex_.z_};

        double t_1, t_2;
        double t_near = std::numeric_limits<double>::min();
        double t_far = std::numeric_limits<double>::max();

        for (size_t i = 0; i < 3; ++i) {
            if (fabs(ray_direction[i]) > std::numeric_limits<double>::epsilon()) {
                t_1 = (minVert[i] - ray_origin[i]) / ray_direction[i];
                t_2 = (maxVert[i] - ray_origin[i]) / ray_direction[i];

                if (t_1 > t_2) {
                    std::swap(t_1, t_2);
                }
                if (t_1 > t_near) {
                    t_near = t_1;
                }
                if (t_2 < t_far) {
                    t_far = t_2;
                }

                if (t_near > t_far) {
                    return std::nullopt;
                }
                if (t_far < 0.) {
                    return std::nullopt;
                }
            } else {
                if (ray_origin[i] < minVertex_.x_ || ray_origin[i] > maxVertex_.x_) {
                    return std::nullopt;
                }
            }
        }

        if (t_near <= t_far && t_far >= 0) {
            return ray.origin_ + ray.direction_ * t_near;
        } else {
            return std::nullopt;
        }
    }

    double Box::DistanceToPoint(const Point &point) const {
        Point centre = Point((minVertex_.x_ + maxVertex_.x_) / 2,
                             (minVertex_.y_ + maxVertex_.y_) / 2,
                             (minVertex_.z_ + maxVertex_.z_) / 2);

        return sqrt(ScalarProduct(Vector(centre, point), Vector(centre, point)));
    }

    Tetrahedron::Tetrahedron(const Point &A, const Point &B,
                             const Point &C, const Point &D) :
            Object('T'), vertices_({A, B, C, D}) {
    }

    const std::array<Point, 4> &Tetrahedron::getVertices() const {
        return vertices_;
    }

    std::optional<Point> Tetrahedron::Intersect(const Ray &ray) const {
        using Edge = std::array<Point, 3>;
        using Intersection = std::optional<double>;

        std::vector<Edge> Edges = {
                {vertices_[0], vertices_[1], vertices_[2]},
                {vertices_[0], vertices_[1], vertices_[3]},
                {vertices_[0], vertices_[2], vertices_[3]},
                {vertices_[1], vertices_[2], vertices_[3]}
        };

        std::vector<Intersection> Intersections(4);
        for (size_t i = 0; i < 3; ++i) {
            Intersections[i] = RayTriangleIntersection(ray, Edges[i]);
        }

        std::vector<double> parameters;
        parameters.reserve(4);
        for (auto &item : Intersections) {
            if (item.has_value()) {
                parameters.push_back(item.value());
            }
        }

        if (parameters.empty()) {
            return std::nullopt;
        } else {
            std::sort(parameters.begin(), parameters.end());
            return ray.origin_ + (ray.direction_ * parameters[0]);
        }
    }

    double Tetrahedron::DistanceToPoint(const Point& point) const {
        std::array<double, 3> sums_ = {
            vertices_[0].x_ + vertices_[1].x_ + vertices_[2].x_ + vertices_[3].x_,
            vertices_[0].y_ + vertices_[1].y_ + vertices_[2].y_ + vertices_[3].y_,
            vertices_[0].z_ + vertices_[1].z_ + vertices_[2].z_ + vertices_[3].z_
        };

        Point centre = Point(sums_[0] / 5., sums_[1] / 5., sums_[2] / 5.);
        return sqrt(ScalarProduct(Vector(centre, point), Vector(centre, point)));
    }

}



std::vector<std::shared_ptr<Geometry3D::Object>> LoadObjects(const std::string& filename) {
    std::ifstream inputFile(filename);
    if (!inputFile) {
        throw std::runtime_error("No input file!");
    }

    std::vector<std::shared_ptr<Geometry3D::Object>> objects;
    objects.reserve(3);

    for (std::string tmp; inputFile >> tmp; ) {
        if (tmp == "sphere") {
            Geometry3D::Point centre;
            double radius;
            inputFile >> centre >> radius;

            objects.push_back(std::make_shared<Geometry3D::Sphere>(centre, radius));
        } else if (tmp == "box") {
            Geometry3D::Point minPoint, maxPoint;
            inputFile >> minPoint >> maxPoint;

            objects.push_back(std::make_shared<Geometry3D::Box>(minPoint, maxPoint));
        } else if (tmp == "tetrahedron") {
            Geometry3D::Point p1, p2, p3, p4;
            inputFile >> p1 >> p2 >> p3 >> p4;

            objects.push_back(std::make_shared<Geometry3D::Tetrahedron>(p1, p2, p3, p4));
        } else {
            throw std::runtime_error("Wrong object type");
        }
    }

    if (objects.size() > 3) {
        throw std::runtime_error("Too many objects");
    }

    return objects;
}
