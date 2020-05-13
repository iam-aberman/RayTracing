//
// Created by Osip on 2020-05-02.
//

#ifndef RAYTRACING_STAGEOBJECTS_H
#define RAYTRACING_STAGEOBJECTS_H

#include "Primitives.h"
#include "Plane.h"

#include <array>
#include <optional>

namespace Geometry {

    class Object {
    public:

        virtual ~Object() = default;

        virtual std::optional<Point> Intersect(const Ray& ray) const = 0;

    };

    template <typename T>
    class Collider : public Object {



    };

    class Sphere : public Object {
    public:

        Sphere() = delete;
        Sphere(const Point& centre, double radius);

        const Point& getCentre() const;
        double getRadius() const;

        std::optional<Point> Intersect(const Ray& ray) const override;


    private:

        Point centre_;
        double radius_;

    };

    class Box : public Object {
    public:

        Box() = delete;
        Box(const Point& minPoint, const Point& maxPoint);

        const std::array<Point, 8>& getVertices() const;
        const std::array<Plane, 6>& getSurfaces() const;

        std::optional<Point> Intersect(const Ray& ray) const override;



    private:

        std::array<Point, 8> vertices_;
        std::array<Plane, 6> surfaces_;

    };

    class Tetrahedron : public Object {
    public:

        Tetrahedron() = delete;
        Tetrahedron(const Point& A, const Point& B,
                    const Point& C, const Point& D);

        const std::array<Point, 4>& getVertices() const;
        const std::array<Plane, 4>& getSurfaces() const;

        std::optional<Point> Intersect(const Ray& ray) const override;


    private:

        std::array<Point, 4> vertices_;
        std::array<Plane, 4> surfaces_;


    };

}


#endif //RAYTRACING_STAGEOBJECTS_H
