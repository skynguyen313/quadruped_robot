#pragma once

#include <cstddef>
#include <array>
#include "quadruped_core/types/leg_id.hpp"

namespace quadruped_core::types
{

constexpr size_t JOINTS_PER_LEG = 3;

using JointVector = std::array<double, JOINTS_PER_LEG>;
using JointArray  = std::array<JointVector, LEG_COUNT>;

}
