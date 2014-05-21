#include <operations/motionoperation.h>
#include <data_types/pole.h>

namespace nav {

///////////////////////////////////////////////////////////////////////
/// Line Follower Motion Operation
///////////////////////////////////////////////////////////////////////
LineFollowerMO::LineFollowerMO(const cv::FileStorage &fs,
                               const bool onRight,
                               const std::shared_ptr<gui> &gui,
                               const std::string& operation)
    : le_(vineyard::LineExtractor(fs)),
      pe_(vineyard::PoleExtractor(fs)),
      on_right_(onRight),
      min_line_size_(fs["lineExtractor"]["minLineSize"]),
      min_head_pole_distance_(fs["lineFollower"]["minHeadPoleDistance"]),
      head_pole_distance_(0.0f),
      head_pole_center_((0.0f,0.0f)),
      line_follower_(fs, fs["lineFollower"]["desiredDistance"], fs["lineFollower"]["desiredTheta"]),
      desired_x_(fs["lineFollower"]["desiredDistance"]),
      desired_theta_(fs["lineFollower"]["desiredTheta"]),
			num_pole_(0),
            tot_num_pole_(10)
{
    GUI_ = gui;
    last_control_ = { 0.0f, 0.0f };
    operation_ = operation;
    last_nearest_setted_ = false;
    end_operation_counter_ = 0;
    min_number_of_end_frames_ = 15; /// TODO: move it to the config file

    head_Pole_kf_.init(2,2);
    cv::setIdentity(head_Pole_kf_.measurementMatrix);
    cv::setIdentity(head_Pole_kf_.processNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(head_Pole_kf_.measurementNoiseCov, cv::Scalar::all(1e-4));
    cv::setIdentity(head_Pole_kf_.errorCovPost, cv::Scalar::all(1));
    cv::setIdentity(head_Pole_kf_.statePost, cv::Scalar::all(0));
}

void LineFollowerMO::updateParameters(const std::shared_ptr<std::vector<vineyard::Pole::Ptr> > &polesVector,
                                      const Control &lastControl,
                                      const float currentBearing,
                                      const float vineyardBearing)
{
//    /// Predict the new position of the head pole
//    // a frame every 20ms
//    float dt = 0.02f;
//    // New robot position after the applied control
//    // r1.x = r0.x + v*cos(theta0)*dt
//    // r1.x = r0.x + v*sin(theta0)*dt
//    // TODO: use odometry
//    cv::Point2f r1(lastControl.linear * std::cos(0) * dt,
//                   lastControl.linear * std::sin(0) * dt);
//    // theta1 = currentBearing
//    cv::Matx22f rotation;
//    float rotationAngle = normalizeAngle_PI(current_bearing_-currentBearing);
//    rotation[0,0] = std::cos(rotationAngle);
//    rotation[0,1] = -std::sin(rotationAngle);
//    rotation[1,0] = std::sin(rotationAngle);
//    rotation[1,1] = std::cos(rotationAngle);

//    cv::Point2f headPolePredicted = (head_pole_center_ - r1) * rotation;
//    /// End prediction

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
        //le_.extractLineFromNearestPole(polesVector, nearest_, line_, true);
        if (operation_.compare("H01L")==0 || operation_.compare("H01R")==0)
        {
        	le_.extractAccessPathFromNearestPole(polesVector, nearest_, line_, true);

            if (last_nearest_setted_ && last_nearest_ID_ != nearest_->ID() &&
                    cv::norm(last_nearest_centroid_ - nearest_->getCentroid()) > 2.0 &&
                    last_nearest_centroid_.x < nearest_->getCentroid().x)
            {
                num_pole_++;

                last_nearest_ID_ = nearest_->ID();
                last_nearest_centroid_ = nearest_->getCentroid();
                last_nearest_setted_ = true;
            }
        	else
        	{
        		if (!last_nearest_setted_)
        			std::cout << "Last è vuoto" << std::endl;
        		else
        		{
                    std::cout << "Last Nearest ID: " << last_nearest_ID_
                              << "      Nearest ID: " << nearest_->ID()
                              << "    Distance: " <<  cv::norm(last_nearest_centroid_-nearest_->getCentroid())
                              << std::endl;
        		}
        	}

        }
        else //TODO: check what is the best option
        	le_.extractLineFromNearestPole(polesVector, nearest_, line_, true);

        if (line_)
        {
            if (GUI_) { GUI_->drawLine(*polesVector,line_,desired_x_); }
            // Check the head pole distance
            int headPoleIndex = line_->getPolesList().front();

            cv::Point2f measuredHeadPoleCenter;
            if (line_->getPolesList().size() > 0)
            {
                measuredHeadPoleCenter = (*polesVector)[headPoleIndex]->getCentroid();

                ///

                head_pole_center_ = measuredHeadPoleCenter;
                head_pole_distance_ = cv::norm(head_pole_center_);
            }

            if (GUI_) { drawPrevPath(); }
        }
    }

    // If line update a kalman filter of the head pole
    //if (line_)
    //{
    head_Pole_kf_.predict();
    cv::Mat measurement = cv::Mat::zeros(2,1,CV_32FC1);
    measurement.at<float>(0) = head_pole_center_.x;
    measurement.at<float>(1) = head_pole_center_.y;
    head_Pole_kf_.correct(measurement);
    std::cout << "measured: " << head_pole_center_;
    head_pole_center_ = cv::Point2f(head_Pole_kf_.statePost.at<float>(0),head_Pole_kf_.statePost.at<float>(1));
    std::cout << " - corrected: " <<  head_pole_center_ << std::endl;

    if (GUI_) { GUI_->drawCross(head_pole_center_); }
    //}

}

void LineFollowerMO::computeErrorXErrorTheta(float &errorX, float &errorTheta,
                                             const Control lastControl,
                                             const cv::Point2f &currentPosition,
                                             const float currentBearing)
{
    //EKFStateEstimator simEKF = ekf_;


    vineyard::LineParams lineParams = line_->getLineParameters();

    float lineAngle = std::atan2(lineParams.vy, lineParams.vx);

    cv::Point2f
            a(lineParams.x0, lineParams.y0),
            b = a + cv::Point2f(lineParams.vx, lineParams.vy);

    float
            numerator = std::abs((b.x-a.x)*(a.y-currentPosition.y)-(a.x-currentPosition.x)*(b.y-a.y)),
            divisor = std::sqrt((b.x-a.x)*(b.x-a.x)+(b.y-a.y)*(b.y-a.y)),
            distance = numerator / divisor;

    errorX = (desired_x_ - distance);
    errorTheta = (on_right_?1:-1) * (desired_theta_ + (currentBearing - lineAngle));
}

void LineFollowerMO::drawPrevPath()
{
    // Setup the simulation variables
    float
            dt = 0.01f,         // Simulation time step
            currentTime = 0.0f,
            maxTime = 25;
    cv::Point2f
            currentPosition(0.0f,0.0f);
    float
            currentBearing = 0.0f;

    // Start the simulation
    while (currentTime < maxTime)
    {
        currentTime = currentTime + dt;

        float
            errorX, errorTheta;

        float
            v = 0.0f,
            omega = 0.0f;

        computeErrorXErrorTheta(errorX, errorTheta, {v, omega},
                                currentPosition,
                                currentBearing);

        v = line_follower_.computeLinearVelocity(errorX, errorTheta);
        omega = line_follower_.computeAngularVelocity(v, errorX, errorTheta);

        currentPosition.x = currentPosition.x + std::cos(currentBearing) * v * dt;
        currentPosition.y = currentPosition.y + std::sin(currentBearing) * v * dt;
        currentBearing = currentBearing + (on_right_?1:-1) * omega * dt;

        if (GUI_) { GUI_->drawPixelPath(currentPosition); /*GUI_->show(1);*/}

        if (v == 0.0f && omega == 0.0f)
        {
            return;
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
        float errorTheta = (on_right_?1:-1) * (desired_theta_ - state_.dtheta);

        // Compute the velocities
        //std::cout << "(ex, et) = (" << errorX << ", " << errorTheta << ")" << std::endl;
        linear = line_follower_.computeLinearVelocity(errorX, errorTheta);
        angular = line_follower_.computeAngularVelocity(linear, errorX, errorTheta);
    }

    return {linear, angular};
}

float LineFollowerMO::checkOperationEnd()
{
	if (operation_.compare("H01L")==0 || operation_.compare("H01R")==0)
	{
        std::cout << num_pole_ << " poles of " << tot_num_pole_ << " - progress:" << (float)num_pole_ / (float)tot_num_pole_ << std::endl;
        return (float)num_pole_ / (float)tot_num_pole_;
    }

    float end = 0.0f;

    float s = 4.0f;

    if (head_pole_center_.x < end)
    {
        end_operation_counter_ = end_operation_counter_ + 1;
        if (end_operation_counter_ >= min_number_of_end_frames_)
        {
            return 1.0f;
        }
        else
        {
            return 0.998f;
        }
    }

    end_operation_counter_ = 0;

    if (head_pole_center_.x > s)
    {
        return 0.0f;
    }

    return 1 - (head_pole_center_.x - end) / (s - end);
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
    final_correction_ = false;
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
                                   float headPoleThreshold,
                                   bool fixedStart)
{
    head_pole_threshold_ = headPoleThreshold;
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
            //std::cout << transform_ << std::endl;

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
    if (GUI_) { drawPrevPath(); }
}

void TurnWithCompassMO::computeRThetaSigma(float &r, float &theta, float &sigma,
                                          const cv::Point2f &robotPosition,
                                          const float &robotBearing)
 {
     float targetDirectionAngle;
     float targetAngle;

     targetDirectionAngle = std::atan2((target_direction_.y - robotPosition.y),
                                       (target_direction_.x - robotPosition.x));
     targetAngle = std::atan2((target_point_.y - robotPosition.y),
                              (target_point_.x - robotPosition.x));

     targetDirectionAngle = normalizeAngle_PI(targetDirectionAngle - robotBearing);
     targetAngle = normalizeAngle_PI(targetAngle);

     r = cv::norm(target_point_ - robotPosition);
     theta = targetDirectionAngle - targetAngle;
     sigma = robotBearing - targetAngle;

     theta = normalizeAngle_PI(theta);
 }

 void TurnWithCompassMO::drawPrevPath()
 {
     // Setup the simulation variables
     float
             dt = 0.01f,         // Simulation time step
             currentTime = 0.0f,
             maxTime = 5;
     cv::Point2f
             currentPosition(0.0f,0.0f);
     float
             currentBearing = 0.0f;

     // Start the simulation
     while (currentTime < maxTime)
     {
         currentTime = currentTime + dt;
         float r,theta,sigma;

         computeRThetaSigma(r, theta, sigma, currentPosition, currentBearing);

         float
             v = 0.0f,
             omega = 0.0f;
         v = u_turn_mp_.computeLinearVelocity(r,theta,sigma);
         omega = u_turn_mp_.computeAngularVelocity(v, r, theta, sigma);

         currentPosition.x = currentPosition.x + std::cos(currentBearing) * v * dt;
         currentPosition.y = currentPosition.y + std::sin(currentBearing) * v * dt;
         currentBearing = currentBearing + omega * dt;

         if (GUI_) { GUI_->drawPixelPath(currentPosition); /*GUI_->show(1);*/}

         if (v == 0.0f && omega == 0.0f || r <= u_turn_mp_.getEndEpsilon())
         {
             return;
         }
     }
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

        std::cout << "punti: " << target_direction_ << " " << target_point_ << std::endl;
        std::cout << "angoli: " << targetDirectionAngle << " " << targetAngle << std::endl;
        std::cout << "r-t-s: " << r_ << " " << theta_ << " " << sigma_ << std::endl;

        linear = u_turn_mp_.computeLinearVelocity(r_,theta_,sigma_);
        angular = u_turn_mp_.computeAngularVelocity(linear,r_,theta_,sigma_);


        if (r_ < end_epsilon_ || final_correction_)
        {
            final_correction_ = true;
            linear = 0.0f;
            float diff = normalizeAngle_PI(theta_ - sigma_);

            if (std::abs(diff) > end_gamma_)
            {
                angular = diff < 0 ? 1.3 : -1.3;
            }
            else
            {
                angular = 0;
            }
        }
    }

    return {linear, angular};
}

float TurnWithCompassMO::checkOperationEnd()
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

    //return r_ <= end_epsilon_ || difference <= end_gamma_;


    float rValue = 0.0f;
    float s_r = 5.0f;

    if (r_ < end_epsilon_)
    {
        rValue = 1.0f;
    }
    else if (r_ > s_r)
    {
        rValue = 0.0f;
    }
    else
    {
        rValue = 1 - (r_ - end_epsilon_) / (s_r - end_epsilon_);
    }

    float gammaValue = 0.0f;
    float s_gamma = M_PI;

    if (difference <= end_gamma_)
    {
        gammaValue = 1.0f;
    }
    else if (difference > s_gamma)
    {
        gammaValue = 0.0f;
    }
    else
    {
        gammaValue = 1 - (difference - end_gamma_) / (s_gamma - end_gamma_);
    }

    return gammaValue * rValue;
}


///////////////////////////////////////////////////////////////////////
/// Special Target Motion Operation
///////////////////////////////////////////////////////////////////////
SpecialTargetMO::SpecialTargetMO(const cv::FileStorage &fs,
                                 const std::string &operationType,
                                 const std::shared_ptr<gui> &gui)
    : operation_type_(operationType),
      target_mp_(EndLineTurnMP(fs)),
      max_v_(fs["globalMP"]["maxV"]),
      end_epsilon_(fs["Uturn"]["endEpsilon"]),
      end_gamma_(fs["Uturn"]["endGamma"]),
      r_(std::numeric_limits<float>::max()),
      theta_(0.0f),
      sigma_(0.0f),
      fixed_pole_threshold_(fs["Uturn"]["headPoleThreshold"])
{
    steered_angle_ = 0.0f;
    GUI_ = gui;
    fixed_pole_ = cv::Point2f(0,0);
    target_point_ = cv::Point2f(0,0);
    target_pole_vec_ = cv::Vec2f(0,0);
    target_bearing_ = 0.0f;
    steered_angle_ = 0.0f;
    start_bearing_ = 0.0f;
    fixed_pole_ID_ = -1;
    final_correction_ = false;
    final_last_linear_vel_ = std::numeric_limits<float>::max();
    set_final_velocity_ = true;
}

void SpecialTargetMO::initialize(const float currentBearing,
                                 const cv::Point2f &fixedPolePosition,
                                 const float targetBearing,
                                 const cv::Vec2f targetPoleVector)
{

    start_bearing_ = currentBearing;

    fixed_pole_ = fixedPolePosition;
    target_bearing_ = target_start_bearing_ = normalizeAngle_PI(currentBearing - targetBearing);
    target_pole_vec_ = targetPoleVector;

    // target vec è un vettore che contiene le coordinate polari del punto target rispetto al punto fisso
    target_point_ = fixed_pole_ + cv::Point2f(target_pole_vec_.val[0] * std::cos(target_pole_vec_.val[1]),
                                              target_pole_vec_.val[0] * std::sin(target_pole_vec_.val[1]));

    if (GUI_) { GUI_->drawHeadPole(fixed_pole_); }
    if (GUI_) { GUI_->drawTarget(target_point_, target_point_ + cv::Point2f(std::cos(target_bearing_),
                                                                            std::sin(target_bearing_))); }
}

void SpecialTargetMO::updateParameters(const std::shared_ptr<std::vector<vineyard::Pole::Ptr> > &polesVector,
                                       const float currentBearing)
{
    steered_angle_ = currentBearing - start_bearing_;
    steered_angle_ = normalizeAngle_PI(steered_angle_);

    //std::cout << steered_angle_ << std::endl;
    bool updateTarget = false;
    vineyard::Pole::Ptr nearest;
    if (fixed_pole_ID_ == -1)
    {
        float minDistanceOrigin = std::numeric_limits<float>::max();
        for (vineyard::Pole_Ptr p : (*polesVector))
        {
            // distance pole-headPole
            float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - fixed_pole_.x,
                                                    p->getCentroid().y - fixed_pole_.y));
            float supposedHeadPoleDistance = cv::norm(p->getCentroid());

            if (distancePH < fixed_pole_threshold_ && supposedHeadPoleDistance < minDistanceOrigin)
            {
                nearest = p;
                fixed_pole_ = p->getCentroid();
                fixed_pole_ID_ = p->ID();
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
            if (p->ID() == fixed_pole_ID_)
            {
                nearest = p;
                fixed_pole_ = p->getCentroid();
                found = true;
                break;
            }
        }
        if (!found)
        {
            /*fixed_pole_ID_ = -1;*/
            float minDistance = std::numeric_limits<float>::max();
            for (vineyard::Pole_Ptr p : (*polesVector))
            {
                // distance pole-headPole
                float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - fixed_pole_.x,
                                                        p->getCentroid().y - fixed_pole_.y));

                if (distancePH < fixed_pole_threshold_ && distancePH < minDistance)
                {
                    nearest = p;
                    fixed_pole_ = p->getCentroid();
                    fixed_pole_ID_ = p->ID();
                    minDistance = distancePH;
                }
            }
        }
        else
        {
            float minDistanceOrigin = std::numeric_limits<float>::max();
            for (vineyard::Pole_Ptr p : (*polesVector))
            {
                // distance pole-headPole
                float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - fixed_pole_.x,
                                                        p->getCentroid().y - fixed_pole_.y));
                float supposedHeadPoleDistance = cv::norm(p->getCentroid());

                if (distancePH < fixed_pole_threshold_ &&
                        supposedHeadPoleDistance < minDistanceOrigin)
                {
                    nearest = p;
                    fixed_pole_ = p->getCentroid();
                    fixed_pole_ID_ = p->ID();
                    minDistanceOrigin = supposedHeadPoleDistance;
                }
            }
        }

        updateTarget = true;
    }

    if (updateTarget)
    {
        // update target point
        // the plus sign is for the inverse orientation of opencv y axis
        float angle = normalizeAngle_PI(target_pole_vec_[1] + steered_angle_);
        target_point_ = fixed_pole_ + cv::Point2f(target_pole_vec_.val[0] * std::cos(angle),
                                                  target_pole_vec_.val[0] * std::sin(angle));

        // update target direction
        // the plus sign is for the inverse orientation of opencv y axis
        target_bearing_ = normalizeAngle_PI(target_start_bearing_ + steered_angle_);
    }

    if (GUI_) { GUI_->drawHeadPole(fixed_pole_); }
    if (GUI_) { GUI_->drawTarget(target_point_, target_point_ + cv::Point2f(std::cos(target_bearing_),
                                                                            std::sin(target_bearing_))); }
    if (GUI_) { drawPrevPath(); }
}

void SpecialTargetMO::computeRThetaSigma(float &r, float &theta, float &sigma,
                                         const cv::Point2f &robotPosition,
                                         const float &robotBearing)
{
    float targetDirectionAngle;
    float targetAngle;

    cv::Point2f targetDirectionPoint(std::cos(target_bearing_) + target_point_.x,
                                     std::sin(target_bearing_) + target_point_.y);

    targetDirectionAngle = std::atan2((targetDirectionPoint.y - target_point_.y),
                                      (targetDirectionPoint.x - target_point_.x));
    targetAngle = std::atan2((target_point_.y - robotPosition.y),
                             (target_point_.x - robotPosition.x));

    targetDirectionAngle = normalizeAngle_PI(targetDirectionAngle);
    targetAngle = normalizeAngle_PI(targetAngle);

    r = cv::norm(target_point_ - robotPosition);
    theta = targetDirectionAngle - targetAngle;
    sigma = robotBearing - targetAngle;

    theta = normalizeAngle_PI(theta);
}

void SpecialTargetMO::drawPrevPath()
{
    // Setup the simulation variables
    float
            dt = 0.01f,         // Simulation time step
            currentTime = 0.0f,
            maxTime = 5;
    cv::Point2f
            currentPosition(0.0f,0.0f);
    float
            currentBearing = 0.0f;

    // Start the simulation
    while (currentTime < maxTime)
    {
        currentTime = currentTime + dt;
        float r,theta,sigma;

        computeRThetaSigma(r, theta, sigma, currentPosition, currentBearing);

        float
            v = 0.0f,
            omega = 0.0f;
        v = target_mp_.computeLinearVelocity(r,theta,sigma);
        omega = target_mp_.computeAngularVelocity(v, r, theta, sigma);

        currentPosition.x = currentPosition.x + std::cos(currentBearing) * v * dt;
        currentPosition.y = currentPosition.y + std::sin(currentBearing) * v * dt;
        currentBearing = currentBearing + omega * dt;

        if (GUI_) { GUI_->drawPixelPath(currentPosition); /*GUI_->show(1);*/}

        if (v == 0.0f && omega == 0.0f || r <= target_mp_.getEndEpsilon())
        {
            return;
        }
    }
}

Control SpecialTargetMO::computeOperationControl()
{
    // Compute the linear and angular velocities
    float linear;
    float angular;
    float targetDirectionAngle;
    float targetAngle;

    targetDirectionAngle = target_bearing_;
    targetAngle = std::atan2(target_point_.y, target_point_.x);

    targetDirectionAngle = normalizeAngle_PI(targetDirectionAngle);
    targetAngle = normalizeAngle_PI(targetAngle);

    r_ = cv::norm(target_point_);
    theta_ = targetDirectionAngle - targetAngle;
    sigma_ = 0 - targetAngle;

    theta_ = normalizeAngle_PI(theta_);

    linear = target_mp_.computeLinearVelocity(r_,theta_,sigma_);
    angular = target_mp_.computeAngularVelocity(linear,r_,theta_,sigma_);

    /// Da provare queste alternative:
    //float progress = checkOperationEnd();
    //float progressThreshold = 0.8f;
    //if (progress >= progressThreshold)
    //{
    //    linear = 0.5 * linear;
    //}

    /// o
    if (r_ <= end_epsilon_ || final_correction_)
    {
        //set_final_velocity_ = false;
        final_correction_ = true;
        r_ = end_epsilon_;
        /// Rallentamento, valutare se serve.
        linear = 0.0;

        float difference = normalizeAngle_PI(theta_ - sigma_);
        std::cout << difference << std::endl;

        angular = difference >= 0 ? 1.5 : -1.5;

        /// Penso che l'errore provenisse da questa riga:
        /// invece di lasciare che si corregga da solo imposta una
        /// velocità angolare fissa che potrebbe essere sbagliata
        //angular = final_last_angular_vel_;
    }

    if (linear < 0.0f)  linear = 0.0f;

    return {linear, angular};
}

float SpecialTargetMO::checkOperationEnd()
{
    // controllo fine operazione
    float difference = std::abs(theta_ - sigma_);
    difference = difference >= 2*M_PI ? difference - 2 * M_PI : difference;
    difference = difference < 0 ? difference + 2 * M_PI : difference;

    difference = std::abs(difference);

    //std::cout << "r_ " << r_ << " - difference " << difference << std::endl;

    //return r_ <= end_epsilon_ || difference <= end_gamma_;


    float rValue = 0.0f;
    float s_r = 5.0f;

    if (r_ < end_epsilon_)
    {
        rValue = 1.0f;
    }
    else if (r_ > s_r)
    {
        rValue = 0.0f;
    }
    else
    {
        rValue = 1 - (r_ - end_epsilon_) / (s_r - end_epsilon_);
    }

    float gammaValue = 0.0f;
    float s_gamma = 2 * M_PI;

    if (difference <= end_gamma_)
    {
        gammaValue = 1.0f;
    }
    else if (difference > s_gamma)
    {
        gammaValue = 0.0f;
    }
    else
    {
        gammaValue = 1 - (difference - end_gamma_) / (s_gamma - end_gamma_);
    }

    if (gammaValue * rValue == 0.0f)
    {
        std::cout << "vvv" << std::endl;
    }

    return gammaValue * rValue;
}

}   // namespace nav
