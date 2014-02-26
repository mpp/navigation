#include "egomotionestimator.h"

namespace nav {

bool simplePoleASCIDSort(const SimplePole &a, const SimplePole &b)
{
    return a.id < b.id;
}

bool pointASCDistanceSort(const cv::Point2f &a, const cv::Point2f &b)
{
    return cv::norm(a) < cv::norm(b);
}

EgoMotionEstimator::EgoMotionEstimator(const int minVectorSize)
    : min_vector_size_(minVectorSize)
{
}

void EgoMotionEstimator::initializePolesVector(const std::shared_ptr<std::vector<vineyard::Pole_Ptr> > &polesVector)
{
    previous_poles_vector_.clear();
    for (vineyard::Pole_ConstPtr p : (*polesVector))
    {
        SimplePole sp = {p->getCentroid(), p->ID()};
        previous_poles_vector_.push_back(sp);
    }

    std::sort(previous_poles_vector_.begin(), previous_poles_vector_.end(), simplePoleASCIDSort);
}

void EgoMotionEstimator::computeRigidTransform(const std::shared_ptr<std::vector<vineyard::Pole_Ptr> > &polesVector,
                                               cv::Matx23f &transform)
{
    // Prepare the current vector
    std::vector< SimplePole > currentPolesVector;
    for (vineyard::Pole_ConstPtr p : (*polesVector))
    {
        SimplePole sp = {p->getCentroid(), p->ID()};
        currentPolesVector.push_back(sp);
    }

    std::sort(currentPolesVector.begin(), currentPolesVector.end(), simplePoleASCIDSort);

    // Extract the common subset
    std::vector< cv::Point2f >
            commonSubsetPrev,
            commonSubsetCurr;

    std::vector< SimplePole >::const_iterator
            prevIT = previous_poles_vector_.begin(),
            currIT = currentPolesVector.begin();

    while (prevIT != previous_poles_vector_.end() &&
           currIT != currentPolesVector.end())
    {
        if (prevIT->id == currIT->id)
        {
            commonSubsetPrev.push_back(prevIT->center);
            commonSubsetCurr.push_back(currIT->center);
            prevIT++;
            currIT++;
        }
        if (prevIT->id < currIT->id)
        {
            prevIT++;
        }
        else
        {
            currIT++;
        }
    }

    // Take only the k nearest points
    int k = 10;
    if (commonSubsetPrev.size() > k)
    {
        std::sort(commonSubsetPrev.begin(), commonSubsetPrev.end(), pointASCDistanceSort);
        std::sort(commonSubsetCurr.begin(), commonSubsetCurr.end(), pointASCDistanceSort);

        commonSubsetPrev.erase(commonSubsetPrev.begin()+k,commonSubsetPrev.end());
        commonSubsetCurr.erase(commonSubsetCurr.begin()+k,commonSubsetCurr.end());
    }

    // Swap the vectors
    previous_poles_vector_.swap(currentPolesVector);

    // Pass the common subset to the rigid transform estimator
    transform = cv::estimateRigidTransform(commonSubsetPrev, commonSubsetCurr, false);

    //std::cout << transform << std::endl;
}

}
