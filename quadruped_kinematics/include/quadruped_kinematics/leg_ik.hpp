#pragma once

#include "quadruped_core/types/joint_types.hpp"
#include "quadruped_core/types/foot_types.hpp"

namespace quadruped_kinematics
{

/**
 * @brief 3-DOF leg inverse kinematics solver.
 *
 * Computes joint angles for a quadruped leg given desired
 * foot positions expressed in the hip coordinate frame (HTF).
 *
 * Kinematic structure:
 *  - Hip abduction/adduction
 *  - Hip pitch
 *  - Knee pitch
 *
 * All link dimensions must be expressed in the same length unit.
 * Returned joint angles are in radians.
 *
 * This class is allocation-free and suitable for real-time control.
 */
class LegIKModel
{
public:
    /**
     * @brief Construct leg IK model with geometric parameters.
     *
     * @param upper  Thigh length (hip pitch → knee)
     * @param lower  Shank length (knee → foot)
     * @param off0   Lateral offset between hip abduction and hip pitch axes
     * @param off1   Vertical offset between hip abduction and hip pitch axes
     */
    LegIKModel(double upper,
               double lower,
               double off0,
               double off1);

    /**
     * @brief Solve inverse kinematics for all legs.
     *
     * @param htf_vecs Foot positions in hip frame.
     *                 Array size must equal LEG_COUNT.
     *
     * @return Joint angles per leg:
     *         {hip_abduction, hip_pitch, knee_pitch}
     *
     * @note
     *  - No joint limits are enforced.
     *  - Targets outside reachable workspace are numerically clamped.
     *  - Function performs no dynamic allocation.
     */
    quadruped_core::types::JointArray
    solve(const quadruped_core::types::FootArray& htf_vecs) const;

private:
    double upper_;   ///< Thigh length
    double lower_;   ///< Shank length
    double off0_;    ///< Hip lateral offset
    double off1_;    ///< Hip vertical offset
};

} // namespace quadruped_kinematics
