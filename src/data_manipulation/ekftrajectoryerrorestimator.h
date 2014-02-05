#ifndef EKFTRAJECTORYERRORESTIMATOR_H
#define EKFTRAJECTORYERRORESTIMATOR_H

#include <opencv2/opencv.hpp>

#include "../data_types/line.h"

namespace nav
{

typedef struct SystemState_
{
    float dy;
    float dtheta;
    float dphi;
} SystemState;

class EKFTrajectoryErrorEstimator
{
public:
    EKFTrajectoryErrorEstimator();

    void setupMeasurementMatrix(const vineyard::LineParams &lineParams,
                                const float compassValue,
                                cv::Mat &measurement);

    void setupControlMatrix(const float linearVelocity,
                            const float angularVelocity,
                            cv::Mat &control);

    void estimate(const cv::Mat &control,
                  const cv::Mat &measurement,
                  SystemState &predicted);

private:
    void computeGJacobian();

    void computeHJacobian();

    cv::KalmanFilter ekf_estimator_;

    cv::Mat ekf_control_;

    cv::Mat ekf_process_noise_;

    cv::Mat ekf_measurement_noise_;

    std::vector<SystemState> ekf_history_;

    float dt_;

    float tx_;
};

} // namespace nav

#endif // EKFTRAJECTORYERRORESTIMATOR_H
