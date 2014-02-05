#include "linefollowermp.h"

LineFollowerMP::LineFollowerMP(const cv::FileStorage &fs,
                               const float desiredX,
                               const float desiredTheta)
    : desired_x_(desiredX),
      desired_theta_(desiredTheta),
      kMaxV(fs["globalMP"]["maxV"]),
      kMaxOmega(fs["globalMP"]["maxOmega"]),
      kXLimitMultiplier(fs["lineFollower"]["limitMultiplier"])
{
    fs["lineFollower"]["a1"] >> a1_;
    fs["lineFollower"]["a2"] >> a2_;
    fs["lineFollower"]["k1"] >> k1_;
    fs["lineFollower"]["k2"] >> k2_;
}

float LineFollowerMP::computeLinearVelocity(const float errorX,
                                            const float errorTheta)
{
    float k = desired_x_ * kMaxV / kXLimitMultiplier;

    return (kMaxV - k * std::abs(errorX) * std::cos(errorTheta));
}

float LineFollowerMP::computeAngularVelocity(const float v,
                                             const float errorX,
                                             const float errorTheta)
{
    float
            k_theta = k1_ / (a1_ + std::abs(errorTheta)),
            k_x = k2_ / (a2_ + std::abs(errorX));

    /*Avoid NAN*/
    if (errorTheta == 0)
    {
        return -1 * k_theta * errorTheta - k_x * errorX * v;
    }

    return -1 * k_theta * errorTheta - k_x * errorX * v * std::sin(errorTheta) / errorTheta;
}
