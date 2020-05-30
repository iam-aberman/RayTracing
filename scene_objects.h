//
// Created by Osip on 2020-05-02.
//

#ifndef RAYTRACING_SCENE_OBJECTS_H
#define RAYTRACING_SCENE_OBJECTS_H

#include "primitives.h"
#include "plane.h"

#include <array>
#include <vector>
#include <optional>
#include <cmath>
#include <string>
#include <memory>

namespace Geometry3D {

    std::optional<Point> RayPlaneIntersection(const Ray& ray, const Plane& plane);
    std::optional<double> RayTriangleIntersection(const Ray& ray,
                                                  const std::array<Point, 3>& vertices);

    class Object {
    public:

        Object(char type = 'O');

        virtual ~Object() = default;
        virtual std::optional<Point> Intersect(const Ray& ray) const = 0;
        virtual double DistanceToPoint(const Point& point) const = 0;

        char getType() const { // Intended to be inline
            return type_;
        }

    private:

        const char type_;

    };

    class Sphere : public Object {
    public:

        Sphere() = delete;
        Sphere(const Point& centre, double radius);

        const Point& getCentre() const;
        double getRadius() const;

        std::optional<Point> Intersect(const Ray& ray) const override;
        double DistanceToPoint(const Point& point) const override;

    private:

        Point centre_;
        double radius_;

    };

    class Box : public Object {
    public:

        Box() = delete;
        Box(const Point& minPoint, const Point& maxPoint);

        std::optional<Point> Intersect(const Ray& ray) const override;
        double DistanceToPoint(const Point& point) const override;

        const Point& getMinVertex() const { // Intended to be inline
            return minVertex_;
        }

        const Point& getMaxVertex() const { // Intended to be inline
            return maxVertex_;
        }

    private:

        const Point minVertex_, maxVertex_;

    };

    class Tetrahedron : public Object {
    public:

        Tetrahedron() = delete;
        Tetrahedron(const Point& A, const Point& B,
                    const Point& C, const Point& D);

        const std::array<Point, 4>& getVertices() const;

        std::optional<Point> Intersect(const Ray& ray) const override;
        double DistanceToPoint(const Point& point) const override;

    private:

        std::array<Point, 4> vertices_;

    };

}

std::vector<std::shared_ptr<Geometry3D::Object>> LoadObjects(const std::string& filename);


#endif //RAYTRACING_SCENE_OBJECTS_H
