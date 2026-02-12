#include "quadruped_kinematics/body_ik.hpp"

namespace quadruped_kinematics {

/**
 * @brief Rigid-body inverse kinematics model for quadruped base.
 *
 * Computes hip-to-foot displacement vectors resulting from body
 * orientation changes. Only rotational effects are considered.
 *
 * Geometry:
 *  - Four hip attachment points defined in body frame
 *  - Four fixed leg base points defined in ground frame
 *
 * All dimensions must use the same length unit.
 */
BodyIKModel::BodyIKModel(double width,
                         double length,
                         double height)
    : width_(width),
      length_(length),
      height_(height)
{
    resetPose();
}

/**
 * @brief Reset body and leg reference geometry.
 *
 * Initializes:
 *  - Body attachment points at nominal height
 *  - Leg base points at ground level (z = 0)
 *
 * No rotation is applied in this configuration.
 */
void BodyIKModel::resetPose()
{
    body_points_ = {{
        {  length_/2, -width_/2, height_ },
        {  length_/2,  width_/2, height_ },
        { -length_/2, -width_/2, height_ },
        { -length_/2,  width_/2, height_ }
    }};

    leg_points_ = {{
        {  length_/2, -width_/2, 0.0 },
        {  length_/2,  width_/2, 0.0 },
        { -length_/2, -width_/2, 0.0 },
        { -length_/2,  width_/2, 0.0 }
    }};
}

/**
 * @brief Apply rigid-body rotation to body attachment points.
 *
 * Rotation order:
 *   yaw (Z) → pitch (Y) → roll (X)
 *
 * Combined rotation:
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *
 * Leg base points remain fixed.
 *
 * @note
 *  - No translation is applied.
 *  - Operation is allocation-free.
 */
void BodyIKModel::transform(double yaw,
                            double pitch,
                            double roll)
{
    const Eigen::Matrix3d Rz =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    const Eigen::Matrix3d Ry =
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();

    const Eigen::Matrix3d Rx =
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();

    const Eigen::Matrix3d R = Rz * Ry * Rx;

    for (auto& p : body_points_)
        p = R * p;
}

/**
 * @brief Compute hip-to-foot displacement vectors.
 *
 * HTF vector definition:
 *   HTF = body_attachment_point - leg_base_point
 *
 * These vectors are typically passed directly to the leg IK solver.
 *
 * @return Fixed-size array of hip-to-foot vectors.
 *
 * @note Realtime-safe. No dynamic allocation.
 */
quadruped_core::types::FootArray BodyIKModel::getHTFVectors() const
{
    quadruped_core::types::FootArray htf{};

    for (size_t i = 0; i < body_points_.size(); ++i)
        htf[i] = body_points_[i] - leg_points_[i];

    return htf;
}

} // namespace quadruped_kinematics
