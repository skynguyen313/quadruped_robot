#pragma once
#include <cstddef>
#include <cstdint>

namespace quadruped_core::types
{

enum class LegID : uint8_t
{
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

constexpr size_t LEG_COUNT = 4;

}