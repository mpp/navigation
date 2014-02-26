#include "endlineturnmp.h"

namespace nav {

EndLineTurnMP::EndLineTurnMP(const cv::FileStorage &fs)
    : k1_(1), k2_(10), k3_(),
    kMaxV(fs["globalMP"]["maxV"])
{
}

float EndLineTurnMP::computeLinearVelocity(const float r,
                                           const float theta,
                                           const float sigma)
{
    float rInv = -1 / r;

    float z = sigma - std::atan(-1 * k1_ * theta);

    curvature_ = rInv * (k2_ * z + (1 + k1_ / (1 + k1_*theta*k1_*theta)) * std::sin(sigma));

    float v = 0.0f;

    if (r < epsilon_)
    {
        v = k3_ * r;
    }
    else
    {
        v = kMaxV / (1 + beta_ * std::pow(std::abs(curvature_), gamma_));
    }

    return v;
}

float EndLineTurnMP::computeAngularVelocity(const float v,
                                            const float r,
                                            const float theta,
                                            const float sigma)
{
    float omega = curvature_ * v;

    return omega;
}

} // namespace nav
