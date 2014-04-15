#include "endlineturnmp.h"

namespace nav {

EndLineTurnMP::EndLineTurnMP(const cv::FileStorage &fs)
    : k1_(fs["Uturn"]["k1"]),
      k2_(fs["Uturn"]["k2"]),
      k3_(fs["Uturn"]["k3"]),
      kMaxV(fs["globalMP"]["maxV"]),
      epsilon_(fs["Uturn"]["endEpsilon"]),
      beta_(fs["Uturn"]["beta"]),
      lambda_(fs["Uturn"]["lambda"])
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

    if (r < 0.2)//epsilon_)
    {
        v = 0.01f;//k3_;
    }
    else
    {
        v = kMaxV / (1 + beta_ * std::pow(std::abs(curvature_), lambda_));
    }

    return v;
}

float EndLineTurnMP::computeAngularVelocity(const float v,
                                            const float r,
                                            const float theta,
                                            const float sigma)
{
    float omega = curvature_ * v;

    float sign = omega>0 ? 1 : -1;

    omega = std::abs(omega)>1.5f ? sign*1.5f : omega;

    return omega;
}

} // namespace nav
