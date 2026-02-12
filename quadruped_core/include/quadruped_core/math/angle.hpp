#pragma once

namespace quadruped_core::math
{

constexpr double PI = 3.14159265358979323846;
constexpr double HALF_PI = PI * 0.5;
constexpr double TWO_PI = PI * 2.0;

inline double deg2rad(double deg)
{
    return deg * PI / 180.0;
}

inline double rad2deg(double rad)
{
    return rad * 180.0 / PI;
}

inline double wrapToPi(double angle)
{
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

}
