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
        else if (prevIT->id < currIT->id)
        {
            prevIT++;
        }
        else
        {
            currIT++;
        }
    }

    // Swap the vectors
    previous_poles_vector_.swap(currentPolesVector);

    if (commonSubsetCurr.size() <= 0 || commonSubsetPrev.size() <= 0)
    {
        return;
    }

    // Take only the k nearest points
    int k = 5;
    if (commonSubsetPrev.size() > k)
    {
        std::sort(commonSubsetPrev.begin(), commonSubsetPrev.end(), pointASCDistanceSort);
        std::sort(commonSubsetCurr.begin(), commonSubsetCurr.end(), pointASCDistanceSort);

        commonSubsetPrev.erase(commonSubsetPrev.begin()+k,commonSubsetPrev.end());
        commonSubsetCurr.erase(commonSubsetCurr.begin()+k,commonSubsetCurr.end());
    }

    // Pass the common subset to the rigid transform estimator
    cv::Mat temp = cv::estimateRigidTransform(commonSubsetPrev, commonSubsetCurr, false);

    if (!temp.empty())
    {
        std::cout << "temp: " << temp << std::endl;

        std::cout << temp.cols << " " << temp.rows << std::endl;

        std::cout << "[ " << temp.at<float>(0,0) << ", " << temp.at<float>(0,1) << ", " << temp.at<float>(0,2)
                  << std::endl
                  << "  " << temp.at<float>(1,0) << ", " << temp.at<float>(1,1) << ", " << temp.at<float>(1,2)
                  << "]" << std::endl;
    }

    if (temp.cols == transform.cols && temp.rows == transform.rows)
    {
        transform = temp;
    }
    std::cout << "[ " << transform(0,0) << ", " << transform(0,1) << ", " << transform(0,2)
              << std::endl
              << "  " << transform(1,0) << ", " << transform(1,1) << ", " << transform(1,2)
              << "]" << std::endl;
    //std::cout << transform << std::endl;
}

}
