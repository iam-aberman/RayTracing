//
// Created by Osip on 2020-04-30.
//

#ifndef RAYTRACING_DOUBLE_COMPARISON_H
#define RAYTRACING_DOUBLE_COMPARISON_H

#include <cmath>

#ifndef EPSILON
#define EPSILON 1e-10

inline bool Equal(double lhs, double rhs) {
    return fabs(lhs - rhs) < EPSILON;
}

#endif

#endif //RAYTRACING_DOUBLE_COMPARISON_H
