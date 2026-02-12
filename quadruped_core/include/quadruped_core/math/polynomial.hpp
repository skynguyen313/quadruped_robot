#pragma once
#include <array>

namespace quadruped_core::math
{

template<size_t N>
inline double evaluatePolynomial(
    const std::array<double, N>& coeff,
    double x)
{
    double result = 0.0;
    for (size_t i = 0; i < N; ++i)
        result = result * x + coeff[i];
    return result;
}

}
