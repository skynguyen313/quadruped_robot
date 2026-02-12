#pragma once
#include <array>

namespace quadruped_core::concepts
{

template<typename T, size_t N>
using FixedArray = std::array<T, N>;

}
