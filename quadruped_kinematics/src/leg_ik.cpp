#include "quadruped_kinematics/leg_ik.hpp"
#include <cmath>
#include <iostream>

namespace quadruped_kinematics {

/**
 * @brief Construct a Leg Inverse Kinematics model
 *
 * @param upper  Length of the thigh link (hip pitch joint → knee joint)
 * @param lower  Length of the shank link (knee joint → foot)
 * @param off0   Lateral offset between hip abduction joint and hip pitch joint
 * @param off1   Vertical offset between hip abduction joint and hip pitch joint
 *
 * @note
 *  - All parameters must be expressed in the same length unit (e.g. mm)
 *  - Joint angles returned by this model are in radians
 */
LegIKModel::LegIKModel(double upper, double lower, double off0, double off1)
    : upper_(upper), lower_(lower), off0_(off0), off1_(off1)
{
}

/**
 * @brief Compute joint angles from hip-to-foot vectors
 *
 * Given a list of foot positions expressed in the hip coordinate frame (HTF),
 * this function computes the corresponding joint angles for each leg.
 *
 * Each input vector (x, y, z) represents the desired foot position relative
 * to the hip abduction joint frame.
 *
 * @param htf_vecs  List of hip-to-foot vectors (x, y, z)
 *
 * @return A vector of joint angle triplets:
 *         {theta_h, theta_s, theta_w}
 *         where:
 *           - theta_h : hip abduction/adduction angle
 *           - theta_s : hip pitch (thigh) angle
 *           - theta_w : knee pitch angle
 *
 * @note
 *  - This model assumes a 3-DOF leg (hip abduction + hip pitch + knee pitch)
 *  - No joint limits or safety clamping are applied
 */
std::vector<std::array<double, 3>>
LegIKModel::ja_from_htf_vecs(const std::vector<std::array<double, 3>>& htf_vecs)
{
    std::vector<std::array<double, 3>> joint_angles;

    try
    {
        for (const auto& vec : htf_vecs)
        {
            // Desired foot position in hip coordinate frame
            double x = vec[0];  // forward/backward
            double y = vec[1];  // lateral
            double z = vec[2];  // vertical

            // Distance between hip abduction joint and hip pitch joint
            double h1 = std::sqrt(off0_ * off0_ + off1_ * off1_);

            // Projection of foot position onto the Y-Z plane
            double h2 = std::sqrt(z * z + y * y);

            // Angle of foot position in Y-Z plane
            double alpha_0 = std::atan(y / z);

            // Fixed geometric angles due to hip joint offsets
            double alpha_1 = std::atan(off1_ / off0_);
            double alpha_2 = std::atan(off0_ / off1_);

            // Law of sines to solve the hip offset triangle
            double alpha_3 = std::asin(
                h1 * std::sin(alpha_2 + HALF_PI) / h2
            );

            // Remaining interior angles of the offset triangle
            double alpha_4 = PI - (alpha_3 + alpha_2 + HALF_PI);
            double alpha_5 = alpha_1 - alpha_4;

            // Hip abduction/adduction angle
            double theta_h = alpha_0 - alpha_5;

            // Effective distance from hip pitch joint to foot projection
            double r0 = h1 * std::sin(alpha_4) / std::sin(alpha_3);
            double h = std::sqrt(r0 * r0 + x * x);

            // Angle between horizontal projection and foot position
            double phi = std::asin(x / h);

            // Hip pitch (thigh) angle using cosine law
            double theta_s =
                std::acos((h * h + upper_ * upper_ - lower_ * lower_) /
                          (2.0 * h * upper_))
                - phi;

            // Knee pitch angle using cosine law
            double theta_w =
                std::acos((lower_ * lower_ + upper_ * upper_ - h * h) /
                          (2.0 * lower_ * upper_));

            // Store joint angles for this leg
            joint_angles.push_back({theta_h, theta_s, theta_w});
        }
    }
    catch (...)
    {
        // Numerical failure (e.g. out-of-reach target causing invalid acos/asin)
        std::cerr << "Leg IK: Out of Bounds." << std::endl;
    }

    return joint_angles;
}

} // namespace quadruped_kinematics