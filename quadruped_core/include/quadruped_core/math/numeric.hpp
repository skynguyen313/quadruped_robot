#pragma once
#include <cmath>

namespace quadruped_core::math
{

inline bool isNearZero(double x, double eps = 1e-9)
{
    return std::abs(x) < eps;
}

inline double square(double x)
{
    return x * x;
}

}
