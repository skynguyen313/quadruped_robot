#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "quadruped_core/types/foot_types.hpp"
#include "quadruped_core/types/leg_id.hpp"

namespace quadruped_kinematics
{

/**
 * @brief Rigid-body inverse kinematics model for quadruped base.
 *
 * Computes hip-to-foot (HTF) displacement vectors resulting from
 * body orientation changes.
 *
 * This class does NOT compute joint angles.
 * It transforms body pose into foot displacement vectors,
 * which are typically passed to LegIKModel.
 *
 * Coordinate convention:
 *  - X: forward
 *  - Y: left
 *  - Z: upward
 *
 * All angles are in radians.
 * All dimensions must use the same length unit.
 *
 * Realtime-safe (no dynamic allocation).
 */
class BodyIKModel
{
public:
    /**
     * @brief Construct body kinematics model.
     *
     * @param width   Distance between left and right legs
     * @param length  Distance between front and rear legs
     * @param height  Nominal body height above ground
     */
    BodyIKModel(double width,
                double length,
                double height);

    /**
     * @brief Reset body and leg reference geometry.
     *
     * Initializes body attachment points at nominal height
     * and leg base points at ground level.
     */
    void resetPose();

    /**
     * @brief Apply rigid-body rotation.
     *
     * Rotation order:
     *   yaw (Z) → pitch (Y) → roll (X)
     *
     * Leg base points remain fixed.
     */
    void transform(double yaw,
                   double pitch,
                   double roll);

    /**
     * @brief Compute hip-to-foot displacement vectors.
     *
     * HTF = body_attachment_point − leg_base_point
     *
     * @return Fixed-size array of HTF vectors.
     */
    quadruped_core::types::FootArray
    getHTFVectors() const;

private:
    double width_;   ///< Body width
    double length_;  ///< Body length
    double height_;  ///< Nominal body height

    quadruped_core::types::FootArray body_points_;
    quadruped_core::types::FootArray leg_points_;

};

} // namespace quadruped_kinematics
