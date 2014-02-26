#ifndef EGOMOTIONESTIMATOR_H
#define EGOMOTIONESTIMATOR_H

#include <algorithm>
#include <memory>
#include <vector>

#include "../data_types/pole.h"

namespace nav {

typedef struct SimplePole_ {
    cv::Point2f center;
    int id;
} SimplePole;

bool simplePoleASCIDSort(const SimplePole &a, const SimplePole &b);
bool pointASCDistanceSort(const cv::Point2f &a, const cv::Point2f &b);

class EgoMotionEstimator
{

public:

    explicit EgoMotionEstimator(const int minVectorSize);

    /*!
     * \brief initializePolesVector
     * \param polesVector
     */
    void initializePolesVector(const std::shared_ptr< std::vector< vineyard::Pole_Ptr > > &polesVector);

    /*!
     * \brief computeRigidTransform
     * \param polesVector
     * \param transform
     */
    void computeRigidTransform(const std::shared_ptr< std::vector< vineyard::Pole_Ptr > > &polesVector,
                               cv::Matx23f &transform);

private:

private:

    std::vector< SimplePole >
        previous_poles_vector_;     //!<

    int min_vector_size_;           //!<

    cv::Matx23f
        rigid_transform_;           //!<

};

} // namespace nav

#endif // EGOMOTIONESTIMATOR_H
