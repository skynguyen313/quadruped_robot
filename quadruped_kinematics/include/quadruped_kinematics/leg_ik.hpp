#pragma once

#include <vector>
#include <array>

namespace quadruped_kinematics {

/**
 * @brief Leg Inverse Kinematics model for a quadruped robot
 *
 * This class computes joint angles (hip, thigh, knee)
 * from HTF (Hip-To-Foot) vectors expressed in the leg coordinate frame.
 *
 * The mathematical model is adapted from the Python implementation
 * and follows a geometric IK approach using trigonometry.
 */
class LegIKModel
{
public:
    /**
     * @brief Constructor
     *
     * @param upper Length of the upper leg link (hip → knee), in meters or mm
     * @param lower Length of the lower leg link (knee → foot), in meters or mm
     * @param off0  Lateral offset from hip joint to leg plane (paper parameter)
     * @param off1  Vertical offset from hip joint to leg plane (paper parameter)
     *
     * off0 and off1 define the hip joint geometry:
     *  - They model the displacement between the hip yaw joint
     *    and the hip pitch joint.
     */
    LegIKModel(double upper, double lower, double off0, double off1);

    /**
     * @brief Compute joint angles from HTF vectors
     *
     * @param htf_vecs Vector of HTF vectors:
     *        Each element is {x, y, z} where:
     *          - x: forward/backward displacement
     *          - y: lateral displacement
     *          - z: vertical displacement (downwards is usually positive)
     *
     * @return std::vector<std::array<double, 3>>
     *         Joint angles for each leg:
     *           [0] hip yaw (or hip roll depending on convention)
     *           [1] hip pitch (shoulder / thigh joint)
     *           [2] knee pitch
     *
     * Notes:
     *  - Output angles are in radians
     *  - The order matches the leg kinematic chain
     *  - Caller is responsible for joint limit checking
     */
    std::vector<std::array<double, 3>>
    ja_from_htf_vecs(const std::vector<std::array<double, 3>>& htf_vecs);

private:
    /// Length of upper leg segment (hip → knee)
    double upper_;

    /// Length of lower leg segment (knee → foot)
    double lower_;

    /// Hip joint horizontal / lateral offset
    double off0_;

    /// Hip joint vertical offset
    double off1_;
};

} // namespace quadruped_kinematics