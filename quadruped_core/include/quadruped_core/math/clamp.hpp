#pragma once
#include <algorithm>

namespace quadruped_core::math
{

template<typename T>
inline T clamp(T value, T min, T max)
{
    return std::max(min, std::min(max, value));
}

}