#include "quadruped_kinematics/body_ik.hpp"

namespace quadruped_kinematics {

using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::AngleAxisd;

/**
 * @brief Construct BodyIKModel with given body dimensions.
 *
 * Initializes body geometry and sets the robot into a neutral pose
 * with no rotation applied.
 *
 * @param width  Body width [mm]
 * @param length Body length [mm]
 * @param height Nominal body height [mm]
 */
BodyIKModel::BodyIKModel(double width, double length, double height)
    : width_(width), length_(length), height_(height) {
    resetPose();
}

/**
 * @brief Reset body and leg reference points to the default pose.
 *
 * Body points represent the hip attachment locations after body rotation.
 * Leg points represent fixed leg base locations on the ground frame.
 *
 * In the reset pose:
 *  - Body points are located at z = height_
 *  - Leg points are located at z = 0
 *
 * No rotation is applied in this state.
 */
void BodyIKModel::resetPose() {
    body_points_.clear();
    leg_points_.clear();

    // Body attachment points (front-right, front-left, rear-right, rear-left)
    body_points_ = {
        { length_/2, -width_/2, height_ },
        { length_/2,  width_/2, height_ },
        { -length_/2, -width_/2, height_ },
        { -length_/2,  width_/2, height_ }
    };

    // Fixed leg base points in the body/world frame
    leg_points_ = {
        { length_/2, -width_/2, 0 },
        { length_/2,  width_/2, 0 },
        { -length_/2, -width_/2, 0 },
        { -length_/2,  width_/2, 0 }
    };
}

/**
 * @brief Apply rigid body rotation to the robot body.
 *
 * Rotates all body attachment points using Euler angles in the following order:
 *  1. Yaw   (Z axis)
 *  2. Pitch (Y axis)
 *  3. Roll  (X axis)
 *
 * The resulting rotation matrix is:
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *
 * Leg base points are NOT transformed, allowing the resulting difference
 * between body and leg points to represent hip-to-foot displacement.
 *
 * @param yaw   Rotation around Z axis [rad]
 * @param pitch Rotation around Y axis [rad]
 * @param roll  Rotation around X axis [rad]
 */
void BodyIKModel::transform(double yaw, double pitch, double roll) {
    // Rotation about Z axis (yaw)
    Matrix3d Rz = AngleAxisd(yaw, Vector3d::UnitZ()).toRotationMatrix();

    // Rotation about Y axis (pitch)
    Matrix3d Ry = AngleAxisd(pitch, Vector3d::UnitY()).toRotationMatrix();

    // Rotation about X axis (roll)
    Matrix3d Rx = AngleAxisd(roll, Vector3d::UnitX()).toRotationMatrix();

    // Combined rotation: yaw -> pitch -> roll
    Matrix3d R = Rz * Ry * Rx;

    // Apply rigid body rotation to all body attachment points
    for (auto &p : body_points_) {
        p = R * p;
    }
}

/**
 * @brief Compute hip-to-foot (HTF) vectors for each leg.
 *
 * HTF vectors are calculated as:
 *   HTF = body_point - leg_point
 *
 * These vectors represent the desired foot position relative to each hip
 * and are typically passed directly to a leg inverse kinematics solver.
 *
 * @return Vector of hip-to-foot vectors, one per leg.
 */
std::vector<Vector3d> BodyIKModel::getHTFVectors() const {
    std::vector<Vector3d> htf;
    for (size_t i = 0; i < body_points_.size(); i++) {
        htf.push_back(body_points_[i] - leg_points_[i]);
    }
    return htf;
}

} // namespace quadruped_kinematics