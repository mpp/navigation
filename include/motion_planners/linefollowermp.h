#ifndef LINEFOLLOWERMP_H
#define LINEFOLLOWERMP_H

#include <opencv2/opencv.hpp>

namespace nav
{

class LineFollowerMP
{
public:
    LineFollowerMP(const cv::FileStorage &fs,
                   const float desiredX,
                   const float desiredTheta);
    /*!
     * @brief computeLinearVelocity u = (u_max - k|error_x|)cos(theta),
     *                              k <= u_max * desired_x / kXLimitMultiplier
     * @param errorX
     * @param errorTheta
     * @return
     */
    float computeLinearVelocity(const float errorX,
                                const float errorTheta);

    /*!
     * @brief computeAngularVelocity omega = -k_phi(phi)*phi - k_errorx(error_x)*error_x*u*sin(phi)/phi,
     *                               k_phi(phi) = k1 / (a1 + |phi|), k_errorx(error_x) = k2 / (a2 + |error_x|)
     * @param v the linear velocity
     * @param errorX
     * @param errorTheta
     * @return
     */
    float computeAngularVelocity(const float v,
                                 const float errorX,
                                 const float errorTheta);


// private data
private:

    float desired_x_;
    float desired_theta_;

    // Angular velocity parameters
    float
        a1_,
        a2_,
        k1_,
        k2_;

// public constants
public:
    const float kMaxV;
    const float kMaxOmega;
    const float kXLimitMultiplier;  //!< right phisical limit of the error_x_ variable

};

} // namespace nav

#endif // LINEFOLLOWERMP_H
