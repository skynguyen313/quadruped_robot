#pragma once

namespace quadruped_core::math
{

class LowPassFilter
{
public:
    LowPassFilter(double alpha)
        : alpha_(alpha)
    {}

    double update(double input)
    {
        state_ = alpha_ * input + (1.0 - alpha_) * state_;
        return state_;
    }

private:
    double alpha_;
    double state_{0.0};
};

}
