//
// Created by Osip on 2020-04-30.
//

#ifndef RAYTRACING_DOUBLECOMPARISON_H
#define RAYTRACING_DOUBLECOMPARISON_H

#include <cmath>

const double EPSILON = 0.0001;

inline bool Equal(double lhs, double rhs) {
    return fabs(lhs - rhs) < EPSILON;
}

#endif //RAYTRACING_DOUBLECOMPARISON_H
