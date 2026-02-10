#pragma once

#include <vector>
#include <array>

namespace quadruped_kinematics {

/**
 * @brief Bezier curve utility for foot swing trajectory generation.
 *
 * This class is a direct C++ translation of the original Python implementation.
 * It is used to generate smooth Bezier trajectories for quadruped gait planning.
 */
class BezierCurve
{
public:
    /**
     * @brief Construct a Bezier curve from control points.
     * @param cp Control points in 2D space: {{x, y}, ...}
     */
    explicit BezierCurve(const std::vector<std::array<double, 2>>& cp);

    /**
     * @brief Compute binomial coefficient (n choose i).
     * @param n Curve degree
     * @param i Term index
     * @return Binomial coefficient value
     */
    static double bin_coeff(int n, int i);

    /**
     * @brief Rotate a vector (x, 0, z) around Z axis.
     * @param x X coordinate
     * @param z Z coordinate
     * @param theta Rotation angle in degrees
     * @return Rotated vector {x, y, z}
     */
    static std::array<double, 3> rotateAboutZ(double x, double z, double theta);

    /**
     * @brief Sample the Bezier curve at parameter t.
     * @param t Curve parameter in range [0, 1]
     * @return Point {x, y} on the Bezier curve
     */
    std::array<double, 2> sample_bezier(double t) const;

    /**
     * @brief Generate default control points from gait parameters.
     * @param L_span Step length
     * @param base_height Nominal foot height
     * @param clearance Foot clearance
     * @return Vector of control points {{x, y}, ...}
     */
    static std::vector<std::array<double, 2>>
    get_cp_from_param(double L_span = 50.0,
                      double base_height = 150.0,
                      double clearance = 0.0);

private:
    /**
     * @brief Factorial helper function.
     * @param n Integer value
     * @return n!
     */
    static double factorial(int n);

private:
    std::vector<std::array<double, 2>> cp_;  ///< Control points
    std::vector<double> coeff_;              ///< Binomial coefficients
    int n_;                                  ///< Bezier curve degree
};

} // namespace quadruped_kinematics