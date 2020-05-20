//
// Created by Osip on 2020-04-30.
//

#ifndef RAYTRACING_DOUBLE_COMPARISON_H
#define RAYTRACING_DOUBLE_COMPARISON_H

#include <cmath>
#include <limits>

inline bool Equal(double lhs, double rhs) {
    return fabs(lhs - rhs) < std::numeric_limits<double>::epsilon();
}

#endif //RAYTRACING_DOUBLE_COMPARISON_H
