#pragma once

namespace quadruped_core::types
{

enum class RobotState
{
    IDLE,
    STAND,
    WALK,
    ERROR,
    EMERGENCY_STOP
};

}
