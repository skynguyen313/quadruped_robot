#include "quadruped_kinematics/leg_ik.hpp"
#include "quadruped_core/math/angle.hpp"
#include "quadruped_core/math/clamp.hpp"
#include "quadruped_core/math/numeric.hpp"
#include <cmath>

namespace quadruped_kinematics {

/**
 * @brief 3-DOF leg inverse kinematics solver.
 *
 * Solves joint angles for a single quadruped leg given a desired
 * foot position expressed in the hip frame (HTF).
 *
 * Geometry assumptions:
 *  - Revolute hip abduction joint
 *  - Revolute hip pitch joint
 *  - Revolute knee pitch joint
 *  - Fixed offsets between hip abduction and hip pitch axes
 *
 * All dimensions must be expressed in the same length unit.
 * All returned joint angles are in radians.
 */
LegIKModel::LegIKModel(double upper,
                       double lower,
                       double off0,
                       double off1)
    : upper_(upper),
      lower_(lower),
      off0_(off0),
      off1_(off1)
{
}

/**
 * @brief Solve inverse kinematics for all legs.
 *
 * @param htf_vecs  Foot positions expressed in the hip frame.
 *                  Array size must match LEG_COUNT.
 *
 * @return Joint angles per leg:
 *         {hip_abduction, hip_pitch, knee_pitch}
 *
 * @note
 *  - No joint limits are enforced.
 *  - No singularity handling beyond basic numeric protection.
 *  - Targets outside reachable workspace are numerically clamped.
 *  - Function is realtime-safe (no dynamic allocation).
 */
quadruped_core::types::JointArray
LegIKModel::solve(const quadruped_core::types::FootArray& htf_vecs) const
{
    quadruped_core::types::JointArray joint_angles{};

    for (size_t i = 0; i < quadruped_core::types::LEG_COUNT; ++i)
    {
        const auto& foot = htf_vecs[i];

        const double x = foot.x();
        const double y = foot.y();
        const double z = foot.z();

        const double h1 = std::sqrt(off0_ * off0_ + off1_ * off1_);

        const double h2 = std::sqrt(z * z + y * y);
        if (h2 < 1e-8)
            continue;

        const double alpha_0 = std::atan2(y, z);

        const double alpha_1 = std::atan2(off1_, off0_);
        const double alpha_2 = std::atan2(off0_, off1_);

        const double sin_arg =
            quadruped_core::math::clamp(h1 * std::sin(alpha_2 + quadruped_core::math::HALF_PI) / h2,
                  -1.0, 1.0);

        const double alpha_3 = std::asin(sin_arg);

        const double alpha_4 =
            quadruped_core::math::PI - (alpha_3 + alpha_2 + quadruped_core::math::HALF_PI);

        const double alpha_5 = alpha_1 - alpha_4;

        const double theta_h = alpha_0 - alpha_5;

        const double sin_alpha3 = std::sin(alpha_3);
        if (std::abs(sin_alpha3) < 1e-8)
            continue;

        const double r0 =
            h1 * std::sin(alpha_4) / sin_alpha3;

        const double h =
            std::sqrt(r0 * r0 + x * x);
        if (h < 1e-8)
            continue;

        const double phi =
            std::asin(quadruped_core::math::clamp(x / h, -1.0, 1.0));

        const double cos_thigh =
            quadruped_core::math::clamp((h * h + upper_ * upper_ - lower_ * lower_) /
                  (2.0 * h * upper_),
                  -1.0, 1.0);

        const double theta_s =
            std::acos(cos_thigh) - phi;

        const double cos_knee =
            quadruped_core::math::clamp((lower_ * lower_ + upper_ * upper_ - h * h) /
                  (2.0 * lower_ * upper_),
                  -1.0, 1.0);

        const double theta_w =
            std::acos(cos_knee);

        joint_angles[i] = {theta_h, theta_s, theta_w};
    }

    return joint_angles;
}

} // namespace quadruped_kinematics
