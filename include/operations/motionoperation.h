#ifndef MOTIONOPERATION_H
#define MOTIONOPERATION_H

#include <opencv2/opencv.hpp>

#include <motion_planners/endlineturnmp.h>
#include <motion_planners/linefollowermp.h>

#include <data_manipulation/egomotionestimator.h>

#include <data_manipulation/ekfstateestimator.h>

#include <data_manipulation/poleextractor.h>
#include <data_manipulation/lineextractor.h>

#include <utils/gui.h>
#include <data_types/pole.h>

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
     * \return percentage (in [0,1]) of the operation progress
     */
    virtual float checkOperationEnd() = 0;

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
                            const std::shared_ptr<gui> &gui = nullptr,
                            const std::string& operation = "");

    /*!
     * \brief initialize
     * \param desiredDistanceFromPole
     */
    void initialize(const float desiredDistanceFromPole) { desired_x_ = desiredDistanceFromPole; }

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

    float checkOperationEnd();

    void getFinalStatus(float &exitLineAngle,
                        cv::Point2f &headPole) const
            { exitLineAngle = std::atan2(line_->getLineParameters().vy, line_->getLineParameters().vx);
              headPole = head_pole_center_;}

private:

    void computeErrorXErrorTheta(float &errorX, float &errorTheta, const Control lastControl,
                                 const cv::Point2f &currentPosition = cv::Point2f(0.0f,0.0f),
                                 const float currentBearing = 0.0f);

    void drawPrevPath();

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
    int
    	num_pole_;
    int
    	tot_num_pole_;
    bool
    	last_nearest_setted_;
    int
    	last_nearest_ID_;
    cv::Point2f
    	last_nearest_centroid_;

    float
        min_head_pole_distance_;
    float
        head_pole_distance_;
    cv::Point2f
        head_pole_center_;

    float
        current_bearing_;

    std::string
    	operation_;

    Control
        last_control_;

    cv::KalmanFilter
        head_Pole_kf_;

    float
        desired_x_, desired_theta_;

    unsigned int
        end_operation_counter_;
    unsigned int
        min_number_of_end_frames_;
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
                    float headPoleThreshold,
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

    float checkOperationEnd() const;

    inline void getLogStatus(float &r,
                             float &theta,
                             float &sigma,
                             cv::Point2f &targetPoint,
                             cv::Point2f &headPole,
                             float &currentLineAngle) const
        {   r = r_; theta = theta_; sigma = sigma_;
            targetPoint = target_point_; headPole = head_pole_;
            if (line_) currentLineAngle = atan2(line_->getLineParameters().vy, line_->getLineParameters().vx);  }

private:

    void computeHeadPole(cv::Point2f initialPole,
                         float forwardDistance,
                         float fixedTurnAngle,
                         float fixedTurnRadius,
                         float exitLineAngle);

    void computeRThetaSigma(float &r, float &theta, float &sigma,
                            const cv::Point2f &robotPosition = cv::Point2f(0.0f,0.0f),
                            const float &robotBearing = 0.0f );

    void drawPrevPath();

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

    bool
        final_correction_;
};


/*!
 * \brief The SpecialTargetMO class set a target point and direction
 */
class SpecialTargetMO : public MotionOperation
{

public:

    /*!
     * \brief SpecialTargetMO
     * \param fs
     * \param operationType la stringa della manovra
     */
    explicit SpecialTargetMO(const cv::FileStorage &fs,
                             const std::string &operationType,
                             const std::shared_ptr<gui> &gui = nullptr);


    /*!
     * \brief initialize
     * \param currentBearing bearing attuale del robot
     * \param fixedPolePosition posizione stimata del palo di riferimento
     * \param targetBearing direzione del target
     * \param targetPoleVector posizione relativa del target rispetto al palo di riferimento
     */
    void initialize(const float currentBearing,
                    const cv::Point2f &fixedPolePosition,
                    const float targetBearing,
                    const cv::Vec2f targetPoleVector);

    /*!
     * \brief updateParameters
     * \param polesVector il vettore dei pali
     * \param lastControl il controllo dato al momento precedente
     * \param currentBearing il bearing attuale
     */
    void updateParameters(const std::shared_ptr< std::vector< vineyard::Pole::Ptr > > &polesVector,
                          const float currentBearing);

    Control computeOperationControl();

    float checkOperationEnd();

    inline void getLogStatus(float &r,
                             float &theta,
                             float &sigma,
                             cv::Point2f &targetPoint,
                             cv::Point2f &fixedPole) const
        {   r = r_; theta = theta_; sigma = sigma_;
            targetPoint = target_point_; fixedPole = fixed_pole_; }

// private methods
private:

    void computeRThetaSigma(float &r, float &theta, float &sigma,
                            const cv::Point2f &robotPosition = cv::Point2f(0.0f,0.0f),
                            const float &robotBearing = 0.0f );
    void drawPrevPath();

// private data
private:

    std::string
        operation_type_;    // oper_t

    nav::EndLineTurnMP
        target_mp_;         // motion planner that work with a target and its direction

    cv::Point2f
        fixed_pole_,        // position of the fixed reference pole
        target_point_;            // position of the target

    cv::Vec2f
        target_pole_vec_;   // relative vector between the fixed pole and the target

    float
        target_bearing_,    // the direction of the target
        target_start_bearing_, // the initial bearing of the target
        steered_angle_,     // the steered angle of the robot from the beginning of the maneuvre
        start_bearing_;     // the initial bearing of the robot

    int
        fixed_pole_ID_;     // the ID of the fixed pole, for tracking

    float
        fixed_pole_threshold_;

    float
        max_v_;             // the maximum velocity of the robot

    float
        end_epsilon_,       // threshold for end operation
        end_gamma_;         // threshold for end operation

    float
        r_,                 // distance from target
        theta_,             // angle target robot-target-direction
        sigma_;             // angle robot robot-target-direction

    bool
        final_correction_,
        set_final_velocity_;
    float
        final_last_linear_vel_,
        final_last_angular_vel_;
};



} //namespace nav

#endif // MOTIONOPERATION_H
