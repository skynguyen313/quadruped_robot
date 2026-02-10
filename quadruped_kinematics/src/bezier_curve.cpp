#include "quadruped_kinematics/bezier_curve.hpp"
#include "quadruped_kinematics/core/math.hpp"
#include <cmath>

namespace quadruped_kinematics {

using quadruped_kinematics::core::deg2rad;

/**
 * @brief BezierCurve constructor implementation.
 */
BezierCurve::BezierCurve(const std::vector<std::array<double, 2>>& cp)
    : cp_(cp)
{
    // Degree of the Bezier curve
    n_ = static_cast<int>(cp_.size()) - 1;

    // Precompute binomial coefficients
    coeff_.reserve(cp_.size());
    for (size_t i = 0; i < cp_.size(); ++i)
    {
        coeff_.push_back(bin_coeff(n_, static_cast<int>(i)));
    }
}

double BezierCurve::bin_coeff(int n, int i)
{
    return factorial(n) /
           (factorial(i) * factorial(n - i));
}

std::array<double, 3>
BezierCurve::rotateAboutZ(double x, double z, double theta)
{
    // Convert degrees to radians
    double rad = deg2rad(theta);

    double cos_t = std::cos(rad);
    double sin_t = std::sin(rad);

    // Manual rotation (equivalent to numpy dot)
    double rx = cos_t * x;
    double ry = sin_t * x;
    double rz = z;

    return {rx, ry, rz};
}

std::array<double, 2>
BezierCurve::sample_bezier(double t) const
{
    double x = 0.0;
    double y = 0.0;

    for (size_t i = 0; i < cp_.size(); ++i)
    {
        double basis =
            coeff_[i] *
            std::pow(1.0 - t, n_ - static_cast<int>(i)) *
            std::pow(t, static_cast<int>(i));

        x += basis * cp_[i][0];
        y += basis * cp_[i][1];
    }

    return {x, y};
}

std::vector<std::array<double, 2>>
BezierCurve::get_cp_from_param(double L_span,
                          double base_height,
                          double clearance)
{
    double x_scaling_factor = L_span / 200.0;
    double y_scaling_factor = base_height / 500.0;
    double x_center = 0.0;

    std::vector<std::array<double, 2>> cp = {
        {x_center - L_span, base_height},
        {x_center - L_span - 80.5 * x_scaling_factor, base_height},
        {x_center - L_span - 100.0 * x_scaling_factor,
         base_height - 138.9 * y_scaling_factor - clearance},
        {x_center - L_span - 100.0 * x_scaling_factor,
         base_height - 138.9 * y_scaling_factor - clearance},
        {x_center - L_span - 100.0 * x_scaling_factor,
         base_height - 138.9 * y_scaling_factor - clearance},
        {x_center,
         base_height - 138.9 * y_scaling_factor - clearance},
        {x_center,
         base_height - 138.9 * y_scaling_factor - clearance},
        {x_center,
         base_height - 178.6 * y_scaling_factor - clearance},
        {x_center + L_span + 103.2 * x_scaling_factor,
         base_height - 178.6 * y_scaling_factor - clearance},
        {x_center + L_span + 103.2 * x_scaling_factor,
         base_height - 178.6 * y_scaling_factor - clearance},
        {x_center + L_span + 82.6 * x_scaling_factor, base_height},
        {x_center + L_span, base_height}
    };

    return cp;
}

double BezierCurve::factorial(int n)
{
    double result = 1.0;
    for (int i = 2; i <= n; ++i)
        result *= i;
    return result;
}

} // namespace quadruped_kinematics