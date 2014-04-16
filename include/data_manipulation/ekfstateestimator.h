#ifndef EKFSTATEESTIMATOR_H
#define EKFSTATEESTIMATOR_H

#include <opencv2/opencv.hpp>

#include <data_types/line.h>

namespace nav
{

typedef struct SystemState_
{
    float dy;
    float dtheta;
    float dphi;
} SystemState;

class EKFStateEstimator
{
public:
    EKFStateEstimator();

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

    void g();

    void h(const cv::Mat &measurement);

    cv::KalmanFilter ekf_estimator_;

    cv::Mat ekf_control_;

    cv::Mat ekf_state_post_;

    cv::Mat ekf_process_noise_;

    cv::Mat ekf_measurement_noise_;

    std::vector<SystemState> ekf_history_;

    float dt_;

    float tx_;
};

} // namespace nav

#endif // EKFSTATEESTIMATOR_H
