#ifndef MOTIONOPERATION_H
#define MOTIONOPERATION_H

#include <opencv2/opencv.hpp>

#include "../motion_planners/endlineturnmp.h"
#include "../motion_planners/linefollowermp.h"

#include "../data_manipulation/egomotionestimator.h"

#include "../data_manipulation/ekfstateestimator.h"

#include "../data_manipulation/poleextractor.h"
#include "../data_manipulation/lineextractor.h"

#include "../utils/gui.h"

namespace nav {

typedef struct Control_
{
    float linear;   //!< linear velocity
    float angular;  //!< angular velocity
} Control;

class MotionOperation
{
public:

    /*!
     * \brief executeOperation execute the designed motion operation
     * \return the control to be applied to the robot
     */
    virtual Control computeOperationControl() = 0;

    /*!
     * \brief checkOperationEnd check if the operation reach its objective
     * \return true/false
     */
    virtual bool checkOperationEnd() const = 0;

protected:

    std::shared_ptr<gui>
        GUI_;

};

class LineFollowerMO : public MotionOperation
{
public:

    /*!
     * \brief LineFollowerMO setup the class with parameters from the config file
     * \param fs the config file
     * \param onRight true if following a line on right
     */
    explicit LineFollowerMO(const cv::FileStorage &fs,
                            const bool onRight,
                            const std::shared_ptr<gui> &gui = nullptr);

    /*!
     * \brief updateParameters update the internal parameters of the class with new data
     * \param polesVector the curretn vector of sensed poles
     * \param lastControl the last control applied to the robot
     */
    void updateParameters(const std::shared_ptr< std::vector< vineyard::Pole::Ptr > > &polesVector,
                          const Control &lastControl,
                          const float currentBearing,
                          const float vineyardBearing);

    Control computeOperationControl();

    bool checkOperationEnd() const;

    void getFinalStatus(float &exitLineAngle,
                        cv::Point2f &headPole) const
            { exitLineAngle = std::atan2(line_->getLineParameters().vy, line_->getLineParameters().vx);
              headPole = head_pole_center_;}

private:

private:

    /// TODO: documentation && test
    nav::LineFollowerMP
        line_follower_;

    vineyard::LineExtractor
        le_;
    vineyard::PoleExtractor
        pe_;
    nav::EKFStateEstimator
        ekf_;

    std::shared_ptr< std::vector< vineyard::Pole::Ptr > >
        poles_vector_;
    vineyard::Pole::Ptr
        nearest_;
    vineyard::Line::Ptr
        line_;
    nav::SystemState
        state_;
    bool
        on_right_;
    int
        min_line_size_;
    float
        min_head_pole_distance_;
    float
        head_pole_distance_;
    cv::Point2f
        head_pole_center_;

    float
        current_bearing_;

    Control
        last_control_;

    float
        desired_x_, desired_theta_;
};

class TurnWithCompassMO : public MotionOperation
{
public:

    /*!
     * \brief LineFollowerMO setup the class with parameters from the config file
     * \param fs the config file
     * \param onRight true if following a line on right
     */
    explicit TurnWithCompassMO(const cv::FileStorage &fs,
                                              const bool onRight,
                                              const std::shared_ptr<gui> &gui = nullptr);

    void initialize(const std::shared_ptr< std::vector< vineyard::Pole::Ptr > > &polesVector,
                    const float currentBearing,
                    cv::Point2f initialPole,
                    float exitLineAngle,
                    float forwardDistance,
                    float fixedTurnAngle,
                    float fixedTurnRadius,
                    bool fixedStart = false);

    /*!
     * \brief updateParameters update the internal parameters of the class with new data
     * \param polesVector the curretn vector of sensed poles
     * \param lastControl the last control applied to the robot
     */
    void updateParameters(const std::shared_ptr< std::vector< vineyard::Pole::Ptr > > &polesVector,
                          const Control &lastControl,
                          const float currentBearing,
                          const float vineyardBearing);

    Control computeOperationControl();

    bool checkOperationEnd() const;

    inline void getLogStatus(float &r,
                             float &theta,
                             float &sigma,
                             cv::Point2f &targetPoint,
                             cv::Point2f &headPole) const
        { r = r_; theta = theta_; sigma = sigma_;
          targetPoint = target_point_; headPole = head_pole_;}

private:

    void computeHeadPole(cv::Point2f initialPole,
                         float forwardDistance,
                         float fixedTurnAngle,
                         float fixedTurnRadius,
                         float exitLineAngle);

private:

    nav::EgoMotionEstimator
        ego_;
    nav::EndLineTurnMP
        u_turn_mp_;

    bool
        ego_initialized_;

    cv::Matx23f
        transform_;

    cv::Point2f
        head_pole_,
        target_point_,
        target_direction_;

    float
        start_bearing_;

    float
        k_; // target distance from line
    int
        head_pole_ID_;

    float
        max_v_;

    float
        head_pole_threshold_;

    float
        steered_angle_;

    bool
        line_follower_;

    float
        end_epsilon_,
        end_gamma_;

    float
        r_,
        theta_,
        sigma_;

    vineyard::PoleExtractor
        pe_;

    vineyard::LineExtractor
        le_;

    vineyard::Line::Ptr
        line_;

    bool
        on_right_;

    bool
        fixed_start_maneuvre_;
};

} //namespace nav

#endif // MOTIONOPERATION_H
