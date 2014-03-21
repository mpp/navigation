#include "motionoperation.h"

namespace nav {

///////////////////////////////////////////////////////////////////////
/// Line Follower Motion Operation
///////////////////////////////////////////////////////////////////////
LineFollowerMO::LineFollowerMO(const cv::FileStorage &fs,
                               const bool onRight,
                               const std::shared_ptr<gui> &gui)
    : le_(vineyard::LineExtractor(fs)),
      pe_(vineyard::PoleExtractor(fs)),
      on_right_(onRight),
      min_line_size_(fs["lineExtractor"]["minLineSize"]),
      min_head_pole_distance_(fs["lineFollower"]["minHeadPoleDistance"]),
      head_pole_distance_(0.0f),
      head_pole_center_((0.0f,0.0f)),
      line_follower_(fs, fs["lineFollower"]["desiredDistance"], fs["lineFollower"]["desiredTheta"]),
      desired_x_(fs["lineFollower"]["desiredDistance"]),
      desired_theta_(fs["lineFollower"]["desiredTheta"])
{
    GUI_ = gui;
    last_control_ = { 0.0f, 0.0f };
}

void LineFollowerMO::updateParameters(const std::shared_ptr<std::vector<vineyard::Pole::Ptr> > &polesVector,
                                      const Control &lastControl,
                                      const float currentBearing,
                                      const float vineyardBearing)
{
    current_bearing_ = currentBearing;

    last_control_.angular = lastControl.angular;
    last_control_.linear = lastControl.linear;

    poles_vector_ = polesVector;


    // Search for the nearest pole left/right
    pe_.findNearestPole(*poles_vector_, on_right_, nearest_);

    // Check if I already have a line to use as reference
    bool useLastLine = false;
    if (line_)
    {
        if (line_->getPolesList().size() > min_line_size_)
        {
            useLastLine = true;
            if (GUI_) { GUI_->drawLastLine(line_); }
        }
    }

    // Extract the line and update the head-pole distance
    if (nearest_)
    {
        //float angoloFilare = 96*M_PI/180;
        float angolo = currentBearing - vineyardBearing;
        if (!line_)
        {
            // Here use the line bearing in place of 0.0f and 1.0f
            angolo = nav::normalizeAngle_PI(angolo);

            vineyard::LineParams fakeParams = {std::cos(angolo), std::sin(angolo),
                                               nearest_->getCentroid().x, nearest_->getCentroid().y,
                                               nearest_->ID(),
                                               nearest_->getCentroid().x, nearest_->getCentroid().y};
            line_ = std::make_shared<vineyard::Line>(polesVector, fakeParams);
            //std::cout << nearest_->ID() << " - " << angolo << " - " << std::cos(angolo) << " - " << std::sin(angolo) << std::endl;

            if (GUI_) { GUI_->drawLastLine(line_,false); }
        }
        //le_.extractLineFromNearestPole(polesVector, nearest_, line_, useLastLine);
        le_.extractLineFromNearestPole(polesVector, nearest_, line_, true);

        if (line_)
        {
            if (GUI_) { GUI_->drawLine(*polesVector,line_); }
            // Check the head pole distance
            int headPoleIndex = line_->getPolesList().front();
            head_pole_center_ = (*polesVector)[headPoleIndex]->getCentroid();
            head_pole_distance_ = cv::norm(head_pole_center_);
        }
    }
}

Control LineFollowerMO::computeOperationControl()
{
    float linear = 0.0f;
    float angular = 0.0f;

    if (line_)
    {
        vineyard::LineParams lineParams = line_->getLineParameters();

        // Update the EKF
        cv::Mat measurement, control;

        ekf_.setupMeasurementMatrix(lineParams, current_bearing_, measurement);
        ekf_.setupControlMatrix(last_control_.linear,last_control_.angular,control);
        ekf_.estimate(control, measurement, state_);

        //std::cout << measurement << std::endl;
        if (GUI_) { GUI_->drawState(state_); }

        // Compute the error
        float errorX = desired_x_ - state_.dy;
        float errorTheta = desired_theta_ - state_.dtheta;

        // Compute the velocities
        linear = line_follower_.computeLinearVelocity(errorX, errorTheta);
        angular = line_follower_.computeAngularVelocity(linear, errorX, errorTheta);
    }

    return {linear, angular};
}

bool LineFollowerMO::checkOperationEnd() const
{
    //std::cout << head_pole_distance_ << " - " << min_head_pole_distance_ << std::endl;
    return (/*head_pole_distance_ < min_head_pole_distance_ &&*/ head_pole_center_.x < 0);
}

///////////////////////////////////////////////////////////////////////
/// Turn with compass Motion Operation
///////////////////////////////////////////////////////////////////////
TurnWithCompassMO::TurnWithCompassMO(const cv::FileStorage &fs,
                                                                   const bool onRight,
                                                                   const std::shared_ptr<gui> &gui)
    : ego_(EgoMotionEstimator(4)),
      u_turn_mp_(EndLineTurnMP(fs)),
      ego_initialized_(false),
      head_pole_ID_(-1),
      max_v_(fs["globalMP"]["maxV"]),
      head_pole_threshold_(fs["Uturn"]["headPoleThreshold"]),
      steered_angle_(0),
      k_(1.5f),
      line_follower_(false),
      end_epsilon_(fs["Uturn"]["endEpsilon"]),
      end_gamma_(fs["Uturn"]["endGamma"]),
      r_(std::numeric_limits<float>::max()),
      theta_(0.0f),
      sigma_(0.0f),
      le_(vineyard::LineExtractor(fs)),
      pe_(vineyard::PoleExtractor(fs)),
      on_right_(onRight)
{
    GUI_ = gui;
    transform_(0,0) = 1.0f;transform_(0,1) = 0.0f;transform_(0,2) = 0.0f;
    transform_(1,0) = 0.0f;transform_(1,1) = 1.0f;transform_(1,2) = 0.0f;
}

void TurnWithCompassMO::computeHeadPole(cv::Point2f initialPole,
                                        float forwardDistance,
                                        float fixedTurnAngle,
                                        float fixedTurnRadius,
                                        float exitLineAngle)
{
    cv::Point2f t; // transform from robot nose at final position to initial position
    t.y = std::sin(exitLineAngle) * forwardDistance + fixedTurnRadius * (1 - std::cos(fixedTurnAngle));
    float a = std::cos(exitLineAngle) * forwardDistance;
    float b = fixedTurnRadius * -1.0 * std::sin(fixedTurnAngle);
    t.x = -1 * (a + b);

    if (!on_right_)
    {
        t.y = -1 * t.y;
    }
    head_pole_ = (initialPole - cv::Point2f(-1.5,0)) + t;
}

void TurnWithCompassMO::initialize(const std::shared_ptr<std::vector<vineyard::Pole::Ptr> > &polesVector,
                                   const float currentBearing,
                                   cv::Point2f initialPole,
                                   float exitLineAngle,
                                   float forwardDistance,
                                   float fixedTurnAngle,
                                   float fixedTurnRadius,
                                   bool fixedStart)
{
    if (!ego_initialized_)
    {
        ego_.initializePolesVector(polesVector);
        ego_initialized_ = true;

        // set manually the head pole, for testing purposes
        //head_pole_ = initialPole;
        computeHeadPole(initialPole, forwardDistance, fixedTurnAngle, fixedTurnRadius, exitLineAngle);

        if (!on_right_)
        {
            target_point_ = head_pole_ + cv::Point2f(k_,0);
            target_direction_ = target_point_ + cv::Point2f(0,-1);
        }
        else
        {
            target_point_ = head_pole_ + cv::Point2f(k_,0);
            target_direction_ = target_point_ + cv::Point2f(0,1);
        }
        start_bearing_ = currentBearing;

        if (fixedStart)
        {
            fixed_start_maneuvre_ = true;
        }
        else
        {
            fixed_start_maneuvre_ = false;
        }

        if (GUI_) { GUI_->drawHeadPole(head_pole_); }
        if (GUI_) { GUI_->drawTarget(target_point_, target_direction_); }
    }
}

void TurnWithCompassMO::updateParameters(const std::shared_ptr<std::vector<vineyard::Pole::Ptr> > &polesVector,
                                                        const Control &lastControl,
                                                        const float currentBearing,
                                                        const float vineyardBearing)
{
    steered_angle_ = currentBearing - start_bearing_;
    steered_angle_ = normalizeAngle_PI(steered_angle_);

    //std::cout << steered_angle_ << std::endl;
    if (fixed_start_maneuvre_ && std::abs(steered_angle_) < M_PI / 2)
    {
        // do nothing, the output velocities are fixed
    }
    else
    {
        fixed_start_maneuvre_ = false;
        bool updateTarget = false;
        vineyard::Pole::Ptr nearest;
        if (head_pole_ID_ == -1)
        {
            if (polesVector->size() > 4)
            {
                ego_.computeRigidTransform(polesVector, transform_);
            }

            float hx = head_pole_.x,
                  hy = head_pole_.y;
            head_pole_.x = transform_(0,0) * hx + transform_(0,1) * hy + transform_(0,2);
            head_pole_.y = transform_(1,0) * hx + transform_(1,1) * hy + transform_(1,2);
            float tx = target_point_.x,
                  ty = target_point_.y;
            target_point_.x = transform_(0,0) * tx + transform_(0,1) * ty + transform_(0,2);
            target_point_.y = transform_(1,0) * tx + transform_(1,1) * ty + transform_(1,2);
            float dx = target_direction_.x,
                  dy = target_direction_.y;
            target_direction_.x = transform_(0,0) * dx + transform_(0,1) * dy + transform_(0,2);
            target_direction_.y = transform_(1,0) * dx + transform_(1,1) * dy + transform_(1,2);

            float minDistanceOrigin = std::numeric_limits<float>::max();
            for (vineyard::Pole_Ptr p : (*polesVector))
            {
                // distance pole-headPole
                float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - head_pole_.x, p->getCentroid().y - head_pole_.y));
                float supposedHeadPoleDistance = cv::norm(p->getCentroid());

                if (distancePH < head_pole_threshold_ && supposedHeadPoleDistance < minDistanceOrigin)
                {
                    nearest = p;
                    head_pole_ = p->getCentroid();
                    head_pole_ID_ = p->ID();
                    minDistanceOrigin = supposedHeadPoleDistance;
                    updateTarget = true;
                }
            }
        }
        else
        {
            bool found = false;
            for (vineyard::Pole_Ptr p : (*polesVector))
            {
                if (p->ID() == head_pole_ID_)
                {
                    nearest = p;
                    head_pole_ = p->getCentroid();
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                head_pole_ID_ = -1;
            }
            else
            {
                float minDistanceOrigin = std::numeric_limits<float>::max();
                for (vineyard::Pole_Ptr p : (*polesVector))
                {
                    // distance pole-headPole
                    float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - head_pole_.x, p->getCentroid().y - head_pole_.y));
                    float supposedHeadPoleDistance = cv::norm(p->getCentroid());

                    if (distancePH < head_pole_threshold_ && supposedHeadPoleDistance < minDistanceOrigin)
                    {
                        nearest = p;
                        head_pole_ = p->getCentroid();
                        head_pole_ID_ = p->ID();
                        minDistanceOrigin = supposedHeadPoleDistance;
                    }
                }
            }

            updateTarget = true;

            /// TODO the magic number must be justified!! It is good only for M_PI rotation.
            /// Instead of this, check if the angle of the line found is above M_PI/2...
            //if (steered_angle_ >= M_PI/5)
            //{
                line_follower_ = true;
            //}
        }

        if (updateTarget)
        {
            if (on_right_)
            {
                target_point_.x = -1 * k_ * std::cos(-M_PI - steered_angle_) + head_pole_.x;
                target_point_.y = k_ * std::sin(-M_PI - steered_angle_) + head_pole_.y;
                target_direction_.x = std::cos(-M_PI/2 - steered_angle_) + target_point_.x;
                target_direction_.y = -1 * std::sin(-M_PI/2 - steered_angle_) + target_point_.y;
            }
            else
            {
                target_point_.x = -1 * k_ * std::cos(M_PI - steered_angle_) + head_pole_.x;
                target_point_.y = k_ * std::sin(M_PI - steered_angle_) + head_pole_.y;
                target_direction_.x = std::cos(M_PI/2 - steered_angle_) + target_point_.x;
                target_direction_.y = -1 * std::sin(M_PI/2 - steered_angle_) + target_point_.y;
            }
        }

        // Update the direction if I can see a line
        if (line_follower_)
        {
            //pe_.findNearestPole(*polesVector, false, nearest);

            if (nearest)
            {
                if (!line_)
                {
                    // Here use the line bearing in place of 0.0f and 1.0f
                    //float angoloFilare = 96*M_PI/180;
                    float angolo = currentBearing - vineyardBearing;
                    angolo = nav::normalizeAngle_PI(angolo);

                    vineyard::LineParams fakeParams = {std::cos(angolo), std::sin(angolo),
                                                       nearest->getCentroid().x, nearest->getCentroid().y,
                                                       nearest->ID(),
                                                       nearest->getCentroid().x, nearest->getCentroid().y};
                    line_ = std::make_shared<vineyard::Line>(polesVector, fakeParams);
                    if (GUI_) { GUI_->drawLastLine(line_,false); }
                }
                //std::cout << nearest->ID() << std::endl;
                le_.extractLineFromNearestPole(polesVector, nearest, line_, true);
                //le_.extractLineFromNearestPole(polesVector, nearest, line_, (line_?true:false));

                if (line_)
                {
                    vineyard::LineParams lineParams = line_->getLineParameters();
                    // check if the line is robust
                    int
                            firstIDX = line_->getPolesList().back(),
                            lastIDX = line_->getPolesList().front();
                    cv::Point2f
                            first = (*polesVector)[firstIDX]->getCentroid(),
                            last = (*polesVector)[lastIDX]->getCentroid();
                    if (cv::norm(first) > cv::norm(last))
                    {
                        cv::Point2f temp = first;
                        first = last;
                        last = temp;
                    }

                    float lineAngle = std::atan2(last.y - first.y, last.x - first.x);
                    if (std::abs(lineAngle) <= M_PI / 2)
                    {
                        // Update the head pole with the nearest pole of the line
                        float minPoleDistance = cv::norm(head_pole_);

                        for (vineyard::PoleIndex idx : line_->getPolesList())
                        {
                            cv::Point2f current = (*polesVector)[idx]->getCentroid();
                            float distance = cv::norm(current);
                            if (distance < minPoleDistance)
                            {
                                head_pole_ = current;
                                minPoleDistance = distance;
                            }
                        }

                        if (GUI_) { GUI_->drawLine(*polesVector,line_); }


                        if (on_right_)
                        {
                            if (lineParams.vy <= 0)
                            {
                                lineParams.vx = -1 * lineParams.vx;
                                lineParams.vy = -1 * lineParams.vy;
                            }
                            target_point_ = head_pole_ + k_ * cv::Point2f(lineParams.vy, -lineParams.vx);
                            target_direction_ = target_point_ + cv::Point2f(lineParams.vx, lineParams.vy);
                        }
                        else
                        {
                            if (lineParams.vy >= 0)
                            {
                                lineParams.vx = -1 * lineParams.vx;
                                lineParams.vy = -1 * lineParams.vy;
                            }
                            target_point_ = head_pole_ - k_ * cv::Point2f(lineParams.vy, -lineParams.vx);
                            target_direction_ = target_point_ + cv::Point2f(lineParams.vx, lineParams.vy);
                        }
                    }
                }
            }
        }
    }

    if (GUI_) { GUI_->drawHeadPole(head_pole_); }
    if (GUI_) { GUI_->drawTarget(target_point_, target_direction_); }
}

Control TurnWithCompassMO::computeOperationControl()
{
    // Compute the linear and angular velocities
    float linear;
    float angular;

    if (fixed_start_maneuvre_)
    {
        angular = 8.41 * 0.35 * (0.65-0.18)/(-2.0*0.6);
        linear = 8.41 * 0.35 * 0.18 - 0.6 * angular;
    }
    else
    {
        float targetDirectionAngle;
        float targetAngle;

        targetDirectionAngle = std::atan2(-1 * target_direction_.y, target_direction_.x);
        targetAngle = std::atan2(-1 * target_point_.y, target_point_.x);

        targetDirectionAngle = normalizeAngle_PI(targetDirectionAngle);
        targetAngle = normalizeAngle_PI(targetAngle);

        r_ = cv::norm(target_point_);
        theta_ = targetDirectionAngle - targetAngle;
        sigma_ = 0 - targetAngle;

        theta_ = normalizeAngle_PI(theta_);

        linear = u_turn_mp_.computeLinearVelocity(r_,theta_,sigma_);
        angular = u_turn_mp_.computeAngularVelocity(linear,r_,theta_,sigma_);
    }

    return {linear, angular};
}

bool TurnWithCompassMO::checkOperationEnd() const
{
    if (fixed_start_maneuvre_)
    {
        return false;
    }
    // controllo fine operazione
    float difference = std::abs(theta_ - sigma_);
    difference = difference >= 2*M_PI ? difference - 2 * M_PI : difference;
    difference = difference < 0 ? difference + 2 * M_PI : difference;

    difference = std::abs(difference);

    //std::cout << "r_ " << r_ << " - difference " << difference << std::endl;

    return r_ <= end_epsilon_ || difference <= end_gamma_;
}

}   // namespace nav
