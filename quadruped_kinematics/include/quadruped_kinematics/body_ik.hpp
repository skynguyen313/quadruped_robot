#pragma once
#include <Eigen/Dense>
#include <vector>

namespace quadruped_kinematics {

/**
 * @class BodyIKModel
 * @brief Quadruped body inverse kinematics model.
 *
 * This class handles the kinematic relationship between the robot body
 * orientation (yaw, pitch, roll) and the resulting hip-to-foot (HTF) vectors
 * for each leg.
 *
 * BodyIK does NOT compute joint angles. Instead, it transforms the rigid body
 * pose into target foot displacement vectors, which are later consumed by
 * LegIK to compute joint angles.
 *
 * Coordinate convention:
 *  - X axis: forward
 *  - Y axis: left
 *  - Z axis: upward
 *
 * Angles are expressed in radians.
 */
class BodyIKModel {
public:
    /**
     * @brief Construct a BodyIKModel.
     *
     * @param width  Body width (distance between left and right legs) [mm]
     * @param length Body length (distance between front and rear legs) [mm]
     * @param height Nominal body height above ground [mm]
     */
    BodyIKModel(double width, double length, double height);

    /**
     * @brief Reset body and leg attachment points to the default neutral pose.
     *
     * This places:
     *  - body points at (±length/2, ±width/2, height)
     *  - leg base points at (±length/2, ±width/2, 0)
     *
     * No rotation is applied in this state.
     */
    void resetPose();

    /**
     * @brief Apply body orientation transformation.
     *
     * Rotates the body rigidly around its center using Euler angles:
     *  - yaw   : rotation around Z axis
     *  - pitch : rotation around Y axis
     *  - roll  : rotation around X axis
     *
     * The rotation order is Z (yaw) → Y (pitch) → X (roll).
     * Only body attachment points are transformed; leg base points remain fixed.
     *
     * @param yaw   Rotation about Z axis [rad]
     * @param pitch Rotation about Y axis [rad]
     * @param roll  Rotation about X axis [rad]
     */
    void transform(double yaw, double pitch, double roll);

    /**
     * @brief Compute hip-to-foot (HTF) vectors for each leg.
     *
     * HTF vectors are defined as:
     *   HTF = body_point - leg_base_point
     *
     * These vectors represent the desired foot positions relative to the hip
     * frame and are typically passed directly to a leg inverse kinematics solver.
     *
     * @return Vector of HTF vectors, one per leg.
     */
    std::vector<Eigen::Vector3d> getHTFVectors() const;

private:
    /// Body width [mm]
    double width_;

    /// Body length [mm]
    double length_;

    /// Nominal body height [mm]
    double height_;

    /// Body attachment points for each leg (after body rotation)
    std::vector<Eigen::Vector3d> body_points_;

    /// Fixed leg base points in the world/body frame
    std::vector<Eigen::Vector3d> leg_points_;
};

} // namespace quadruped_kinematics