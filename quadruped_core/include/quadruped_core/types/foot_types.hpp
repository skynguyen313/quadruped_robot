#pragma once

#include <Eigen/Core>
#include <array>
#include "quadruped_core/types/leg_id.hpp"

namespace quadruped_core::types
{

using FootPosition = Eigen::Vector3d;
using FootArray    = std::array<FootPosition, LEG_COUNT>;

}
