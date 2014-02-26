#ifndef EGOMOTIONESTIMATOR_H
#define EGOMOTIONESTIMATOR_H

#include <memory>
#include <vector>

#include "../data_types/pole.h"

namespace nav {

class EgoMotionEstimator
{

public:

    EgoMotionEstimator();

private:



private:

    std::shared_ptr< std::vector< vineyard::Pole_Ptr > >
        current_poles_vector_;

    int min_vector_size_;

    cv::Matx23f
        rigid_transform_;

};

} // namespace nav

#endif // EGOMOTIONESTIMATOR_H
