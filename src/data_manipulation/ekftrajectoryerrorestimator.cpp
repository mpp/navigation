#include "ekftrajectoryerrorestimator.h"

namespace nav
{

EKFTrajectoryErrorEstimator::EKFTrajectoryErrorEstimator()
{
    // 3 dinamic parameters: y, theta, phi
    // 3 measurement parameters: y, theta, phi
    // 2 control parameters: v, omega
    ekf_estimator_.init(3,3,2);

    ekf_process_noise_ = cv::Mat::zeros(3, 3, CV_32F);
    ekf_measurement_noise_ = cv::Mat::zeros(3, 3, CV_32F);

    /// TODO: set the initial statePost, how??
}

void EKFTrajectoryErrorEstimator::setupControlMatrix(const float linearVelocity,
                                                     const float angularVelocity,
                                                     cv::Mat &control)
{
    control = cv::Mat::zeros(2,1,CV_32F);
    control.at<float>(0) = linearVelocity;
    control.at<float>(1) = angularVelocity;
}

void EKFTrajectoryErrorEstimator::setupMeasurementMatrix(const vineyard::LineParams &lineParams,
                                                         const float compassValue,
                                                         cv::Mat &measurement)
{
    ///0
    ///|>====D TODO: use the code in the lab PC to complete this matrix
    ///0
    measurement = cv::Mat::zeros(3,1,CV_32F);
    measurement.at<float>(0) = 0.0;
    measurement.at<float>(1) = 0.0;
    measurement.at<float>(2) = compassValue;
}

void EKFTrajectoryErrorEstimator::computeGJacobian()
{
    float theta = ekf_estimator_.statePost.at<float>(1);
    float v = ekf_control_.at<float>(0);
    ekf_estimator_.transitionMatrix =
            *(cv::Mat_<float>(3,3) << 1, dt_ * v * std::cos(theta)), 0,
                                      0, 1, 0,
                                      0, 0, 1;
}

void EKFTrajectoryErrorEstimator::computeHJacobian()
{
    float theta = ekf_estimator_.statePost.at<float>(1);
    ekf_estimator_.measurementMatrix =
            *(cv::Mat_<float>(3,3) << 1, tx_ * std::cos(theta)), 0,
                                      0, 1, 0,
                                      0, 1, 1;
}

void EKFTrajectoryErrorEstimator::estimate(const cv::Mat &control,
                                           const cv::Mat &measurement,
                                           SystemState &predicted)
{
    control.copyTo(ekf_control_);
    computeGJacobian();
    computeHJacobian();

    ekf_estimator_.predict();
    ekf_estimator_.correct(measurement);

    predicted.dy = ekf_estimator_.statePre.at<float>(0);
    predicted.dtheta = ekf_estimator_.statePre.at<float>(1);
    predicted.dphi = ekf_estimator_.statePre.at<float>(2);
}

} // namespace nav
