#include "ekfstateestimator.h"

namespace nav
{

EKFStateEstimator::EKFStateEstimator()
{
    // 3 dinamic parameters: y, theta, phi
    // 3 measurement parameters: y, theta, phi
    // 2 control parameters: v, omega
    ekf_estimator_.init(3,3,2);

    ekf_process_noise_ = cv::Mat::zeros(3, 3, CV_32F);
    ekf_process_noise_.at<float>(0,0) = 0.1;
    ekf_process_noise_.at<float>(0,1) = 0.05;
    ekf_process_noise_.at<float>(1,1) = 0.2;
    ekf_process_noise_.at<float>(1,0) = 0.05;
    ekf_process_noise_.at<float>(2,2) = 0.1;

    ekf_measurement_noise_ = cv::Mat::zeros(3, 3, CV_32F);
    ekf_measurement_noise_.at<float>(0,0) = 0.1;
    ekf_measurement_noise_.at<float>(0,1) = 0.05;
    ekf_measurement_noise_.at<float>(1,1) = 0.2;
    ekf_measurement_noise_.at<float>(1,0) = 0.05;
    ekf_measurement_noise_.at<float>(2,2) = 0.1;

    tx_ = 1;
    dt_ = 0.01;

    /// TODO: set the initial statePost, how?? [1.50875592; 0.31273764; 1.79768908]
}

void EKFStateEstimator::setupControlMatrix(const float linearVelocity,
                                           const float angularVelocity,
                                           cv::Mat &control)
{
    control = cv::Mat::zeros(2,1,CV_32F);
    control.at<float>(0) = linearVelocity;
    control.at<float>(1) = angularVelocity;
}

void EKFStateEstimator::setupMeasurementMatrix(const vineyard::LineParams &lineParams,
                                                         const float compassValue,
                                                         cv::Mat &measurement)
{
    /// TODO: consider the offset between the laser and the center of the robot
    // I have to measure the distance of the line from the origin

    float divisor = std::sqrt(lineParams.vx * lineParams.vx + lineParams.vy * lineParams.vy);
    float numerator = lineParams.x0 * lineParams.vy - lineParams.y0 * lineParams.vx;
    float d = std::abs(numerator / divisor);

    float theta = std::atan2(lineParams.vy, lineParams.vx);

    if (std::isnan(d))
    {
        d = 0;
    }

    measurement = cv::Mat::zeros(3,1,CV_32F);
    measurement.at<float>(0) = d;
    measurement.at<float>(1) = theta;
    measurement.at<float>(2) = compassValue;
}

void EKFStateEstimator::g()
{
    float theta = ekf_state_post_.at<float>(1);
    float y = ekf_state_post_.at<float>(0);
    float phi = ekf_state_post_.at<float>(2);
    float v = ekf_control_.at<float>(0);
    float omega = ekf_control_.at<float>(1);

    // compute the prediction
    ekf_estimator_.statePre =
            *(cv::Mat_<float>(3,1) << y + dt_ * v * std::sin(theta),
                                      theta + dt_ * omega,
                                      phi);

    ekf_estimator_.statePre.copyTo(ekf_estimator_.statePost);
}

void EKFStateEstimator::h(const cv::Mat &measurement)
{
    float theta = ekf_state_post_.at<float>(1);
    float y = ekf_state_post_.at<float>(0);
    float phi = ekf_state_post_.at<float>(2);
    float v = ekf_control_.at<float>(0);
    float omega = ekf_control_.at<float>(1);

    cv::Mat H = *(cv::Mat_<float>(3,1) << y + tx_ * std::sin(theta),
                                          theta,
                                          phi );

    ekf_estimator_.temp5 = measurement - H;

    ekf_estimator_.statePost = ekf_estimator_.statePre - ekf_estimator_.gain * ekf_estimator_.temp5;
}

void EKFStateEstimator::computeGJacobian()
{
    float theta = ekf_estimator_.statePost.at<float>(1);
    float v = ekf_control_.at<float>(0);
    ekf_estimator_.transitionMatrix =
            *(cv::Mat_<float>(3,3) << 1, dt_ * v * std::cos(theta), 0,
                                      0, 1, 0,
                                      0, 0, 1);
//    std::cout << "Transition matrix: " << std::endl;
//    std::cout << ekf_estimator_.transitionMatrix << std::endl;
}

void EKFStateEstimator::computeHJacobian()
{
    float theta = ekf_estimator_.statePost.at<float>(1);
    ekf_estimator_.measurementMatrix =
            *(cv::Mat_<float>(3,3) << 1, tx_ * std::cos(theta), 0,
                                      0, 1, 0,
                                      0, 1, 1);
}

void EKFStateEstimator::estimate(const cv::Mat &control,
                                 const cv::Mat &measurement,
                                 SystemState &predicted)
{
    // 0 - save in a temp matrix the state post and save the control matrix
    ekf_estimator_.statePost.copyTo(ekf_state_post_);
//    std::cout << "StatePost of the previous estimation: " << std::endl;
//    std::cout << ekf_state_post_ << std::endl;

    control.copyTo(ekf_control_);
//    std::cout << "Control Matrix: " << std::endl;
//    std::cout << ekf_control_ << std::endl;
//    std::cout << "Measurement Matrix: " << std::endl;
//    std::cout << measurement << std::endl;

    // 1 - put the Jacobian of G in the transition matrix
    computeGJacobian();
//    std::cout << "Transition matrix: " << std::endl;
//    std::cout << ekf_estimator_.transitionMatrix << std::endl;

    // 2 - put the Jacobian of H in the measurement matrix
    computeHJacobian();
//    std::cout << "Measurement matrix: " << std::endl;
//    std::cout << ekf_estimator_.measurementMatrix << std::endl;

    // 3 - do the prediction step
    ekf_estimator_.predict();
//    std::cout << "Transition covariance matrix: " << std::endl;
//    std::cout << ekf_estimator_.errorCovPre << std::endl;

    // 4 - correct the statePre variable with the non linear g function
    g();
//    std::cout << "Predicted state: " << std::endl;
//    std::cout << ekf_estimator_.statePre << std::endl;

    // 5 - do the correction step
    ekf_estimator_.correct(measurement);
//    std::cout << "Measurement covariance matrix: " << std::endl;
//    std::cout << ekf_estimator_.errorCovPost << std::endl;

    // 6 - correct the temp5 matrix
    h(measurement);
    ekf_estimator_.correct(measurement);
//    std::cout << "Estimated state: " << std::endl;
//    std::cout << ekf_estimator_.statePost << std::endl;

    predicted.dy = ekf_estimator_.statePost.at<float>(0);
    predicted.dtheta = ekf_estimator_.statePost.at<float>(1);
    predicted.dphi = ekf_estimator_.statePost.at<float>(2);
}

} // namespace nav
