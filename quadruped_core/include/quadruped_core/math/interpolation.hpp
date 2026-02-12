#pragma once

namespace quadruped_core::math
{

inline double lerp(double a, double b, double t)
{
    return a + (b - a) * t;
}

inline double cubic(double t)
{
    return 3*t*t - 2*t*t*t;
}

inline double quintic(double t)
{
    return 6*t*t*t*t*t - 15*t*t*t*t + 10*t*t*t;
}

}
