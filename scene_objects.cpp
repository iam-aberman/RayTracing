//
// Created by Osip on 2020-05-02.
//

#include "scene_objects.h"
#include "utilities.h"

#include <cmath>
#include <algorithm>
#include <vector>

namespace Geometry3D {

    std::optional<Point> RayPlaneIntersection(const Ray& ray, const Plane& plane) {
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

    std::optional<double> RayTriangleIntersection(const Ray& ray,
                                                 const std::array<Point, 3>& vertices) {
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
            std::cerr << "U" << std::endl;
            return std::nullopt;
        }

        Vector v_2 = CrossProduct(v1, E1);
        double v = ScalarProduct(ray.direction_, v_2) * invertedDet;
        if (v < 0 || u + v > 1) {
            std::cerr << "V" << std::endl;
            return std::nullopt;
        }

        double t = ScalarProduct(E2, v_2) * invertedDet;
        // return ray.origin_ + (ray.direction_ * t);
        return t;
    }

    Sphere::Sphere(const Point& centre, double radius) :
    centre_(centre),
    radius_(radius)
    {
    }

    const Point& Sphere::getCentre() const {
        return centre_;
    }

    double Sphere::getRadius() const {
        return radius_;
    }

    std::optional<Point> Sphere::Intersect(const Ray& ray) const {
        Vector sphere_to_origin = ray.origin_ - centre_;

        double B = 2. * ScalarProduct(sphere_to_origin, ray.direction_);
        double C = ScalarProduct(sphere_to_origin, sphere_to_origin) - radius_ * radius_;
        double discriminant = B * B - 4 * C;

        if (discriminant < 0) {
            return std::nullopt;
        } else {
            const double discriminantRoot = sqrt(discriminant);
            double t_1 = (- B + discriminantRoot) / 2.;
            double t_2 = (- B - discriminantRoot) / 2.;

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

    Box::Box(const Point& minPoint, const Point& maxPoint) : minVertex_(minPoint),
                                                             maxVertex_(maxPoint)
    {
    }

    std::optional<Point> Box::Intersect(const Ray& ray) const {
        const auto ray_origin = ray.origin_.getNums();
        const auto ray_direction = ray.direction_.getNums();

        if (minVertex_.x_ <= ray_origin[0] && ray_origin[0] <= maxVertex_.x_ &&
            minVertex_.y_ <= ray_origin[1] && ray_origin[1] <= maxVertex_.y_ &&
            minVertex_.z_ <= ray_origin[2] && ray_origin[2] <= maxVertex_.z_) {
            throw std::runtime_error("Camera inside an object!");
        }

        double t_1, t_2;
        double t_near = std::numeric_limits<double>::min();
        double t_far  = std::numeric_limits<double>::max();

        for (size_t i = 0; i < 3; ++i) {
            if (fabs(ray_direction[i]) > std::numeric_limits<double>::epsilon()) {
                t_1 = (minVertex_.x_ - ray_origin[i]) / ray_direction[i];
                t_2 = (maxVertex_.x_ - ray_origin[i]) / ray_direction[i];

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

    Tetrahedron::Tetrahedron(const Point& A, const Point& B,
                             const Point& C, const Point& D) :
                             vertices_({A, B, C, D}) {
        surfaces_[0] = Plane(vertices_[0], vertices_[1], vertices_[2]);
        surfaces_[1] = Plane(vertices_[1], vertices_[2], vertices_[3]);
        surfaces_[0] = Plane(vertices_[0], vertices_[2], vertices_[3]);
        surfaces_[0] = Plane(vertices_[0], vertices_[1], vertices_[3]);
    }

    const std::array<Point, 4>& Tetrahedron::getVertices() const {
        return vertices_;
    }

    const std::array<Plane, 4>& Tetrahedron::getSurfaces() const {
        return surfaces_;
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
        parameters.reserve(4u);
        for (auto& item : Intersections) {
            std::cerr << item.has_value() << std::endl;
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

    /* std::optional<Point> RayBox(const Box& box, const Ray& ray) {
        const auto ray_origin = ray.origin_.getNums();
        const auto ray_direction = ray.direction_.getNums();

        if (box.minVertex_.x_ <= ray_origin[0] && ray_origin[0] <= box.maxVertex_.x_ &&
                box.minVertex_.y_ <= ray_origin[1] && ray_origin[1] <= box.maxVertex_.y_ &&
                box.minVertex_.z_ <= ray_origin[2] && ray_origin[2] <= box.maxVertex_.z_) {
            throw std::runtime_error("Camera inside an object!");
        }

        double t_1, t_2;
        double t_near = std::numeric_limits<double>::min();
        double t_far  = std::numeric_limits<double>::max();

        for (size_t i = 0; i < 3; ++i) {
            if (fabs(ray_direction[i]) > std::numeric_limits<double>::epsilon()) {
                t_1 = (box.minVertex_.x_ - ray_origin[i]) / ray_direction[i];
                t_2 = (box.maxVertex_.x_ - ray_origin[i]) / ray_direction[i];

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
                if (ray_origin[i] < box.minVertex_.x_ || ray_origin[i] > box.maxVertex_.x_) {
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
*/
}
