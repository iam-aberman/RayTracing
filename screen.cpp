//
// Created by Osip Chin on 20.05.2020.
//

#include "screen.h"
#include "utilities.h"

#include <cmath>

#ifndef PI
#define PI 3.14159265

using namespace Geometry3D;

void func(const Point& camera, double distance, double alpha,
          Vector& normal, Vector& yAxis, size_t screenHeight, size_t screenWidth) {
    normal.Normalize();
    yAxis.Normalize();

    const Point screenCentre = camera + normal * distance;
    const Vector xAxis = CrossProduct(normal, yAxis);

    alpha = alpha * PI / 180.; // Degrees to radians
    double height_half = distance / tan(alpha);

    double pixel_dimension = 2 * height_half / screenHeight;
    Point minScreenPoint = screenCentre + (-xAxis * (screenWidth / 2. - 1));
          minScreenPoint = minScreenPoint + (-yAxis * (screenHeight / 2. - 1));

}

#endif // PI