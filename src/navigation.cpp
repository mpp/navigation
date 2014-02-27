#include <iostream>

#include <opencv2/opencv.hpp>

#include "sensors_drivers/logparser.h"

#include "data_manipulation/poleextractor.h"
#include "data_manipulation/lineextractor.h"

#include "data_manipulation/ekfstateestimator.h"

#include "data_manipulation/egomotionestimator.h"

#include "motion_planners/linefollowermp.h"
#include "motion_planners/endlineturnmp.h"

#include "utils/gui.h"

#define LINE_FOLLOWER_
#define EGOMOTION_ESTIMATION_

void help();

//////
/// RANDOM NUMBER GENERATOR
const double DBL_EPS_COMP = 1 - DBL_EPSILON; // DBL_EPSILON is defined in <limits.h>.
inline double RandU() {
    return DBL_EPSILON + ((double) rand()/RAND_MAX);
}
inline double RandN2(double mu, double sigma) {
    return mu + (rand()%2 ? -1.0 : 1.0)*sigma*pow(-log(DBL_EPS_COMP*RandU()), 0.5);
}
inline double RandN() {
    return RandN2(0, 0.1);
}
///
//////

int main(int argc, char **argv)
{
    //std::cout << "Hi" << std::endl;

    //////
    /// Check arguments and set the locale
    if (argc != 3)
    {
        help();
    }

    if (std::string(argv[1]) != "-s")
    {
        help();
    }

    std::cout << std::fixed << std::setprecision(6) /*<< "Hello!"*/ << std::endl;
    setlocale(LC_NUMERIC, "C");

    //////
    /// Open config file
    std::string
        settfilename = argv[2];

    cv::FileStorage
        fs;

    fs.open(settfilename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cerr << "Could not open settings file: " << settfilename << std::endl;
        exit(-2);
    }

    nav::gui GUI(fs);

    cv::Mat image;
    cv::namedWindow("navigation gui");

    int waitkey;
    fs["logparser"]["waitkey"] >> waitkey;
    int minLineSize;
    fs["lineExtractor"]["minLineSize"] >> minLineSize;
    float minHeadPoleDistance;
    fs["lineFollower"]["minHeadPoleDistance"] >> minHeadPoleDistance;

    char c = 0;
    //float angle = 0;
    //float angleIncrement = M_PI / 8;

    std::vector<nav::Frame> framesVector;

    nav::parseFile(fs, framesVector);

    std::cout << framesVector.size() << std::endl;

    vineyard::PoleExtractor pe(fs);
    std::shared_ptr< std::vector< vineyard::Pole::Ptr > > polesVector;

#ifdef LINE_FOLLOWER_
    vineyard::LineParams lineParams;
    vineyard::LineExtractor le(fs);

    nav::EKFStateEstimator ekf;

    vineyard::Line::Ptr line;

    bool lineFollower = false;
#endif
#ifdef EGOMOTION_ESTIMATION_
    nav::EgoMotionEstimator ego(4);
    nav::EndLineTurnMP UTurnMP(fs);
    bool egoInitialized = false;
    cv::Matx23f transform;
    cv::Point2f headPole, targetPoint, targetDirection;
    float startBearing;
    float k = 1.5f; // target distance from line
    int headPoleID = -1;

    std::cout << "targetDirectionAngle;targetAngle;r;theta;sigma;linear;angular" << std::endl;
#endif

    for (nav::Frame f : framesVector)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr
                tempCloud(new pcl::PointCloud<pcl::PointXYZ>());
        for (nav::PT pt : f.points)
        {
            pcl::PointXYZ current;
            current.x = pt.scan_pt.x;
            current.y = pt.scan_pt.y;
            current.z = 0;

            tempCloud->points.push_back(current);
        }
        tempCloud->width = (int) tempCloud->points.size ();
        tempCloud->height = 1;

        pe.elaborateCloud(tempCloud, polesVector);

#ifdef LINE_FOLLOWER_
        vineyard::Pole::Ptr nearest;
        pe.findNearestPole(*polesVector, false, nearest);
#endif

#ifdef EGOMOTION_ESTIMATION_

        if (f.frameID == 430)
        {
            std::cout << "debug" << std::endl;
        }
        if (!egoInitialized)
        {
            ego.initializePolesVector(polesVector);
            egoInitialized = true;
            // set manually the head pole, for testing purposes
            headPole = (*polesVector)[1]->getCentroid();
            targetPoint = (*polesVector)[1]->getCentroid() + cv::Point2f(0,-k);
            targetDirection = targetPoint + cv::Point2f(-1,0);
            startBearing = f.bearing;
        }
        else
        {
            if (headPoleID == -1)
            {
                ego.computeRigidTransform(polesVector, transform);

                float hx = headPole.x,
                      hy = headPole.y;
                headPole.x = transform(0,0) * hx + transform(0,1) * hy + transform(0,2);
                headPole.y = transform(1,0) * hx + transform(1,1) * hy + transform(1,2);
                float tx = targetPoint.x,
                      ty = targetPoint.y;
                targetPoint.x = transform(0,0) * tx + transform(0,1) * ty + transform(0,2);
                targetPoint.y = transform(1,0) * tx + transform(1,1) * ty + transform(1,2);
                float dx = targetDirection.x,
                      dy = targetDirection.y;
                targetDirection.x = transform(0,0) * dx + transform(0,1) * dy + transform(0,2);
                targetDirection.y = transform(1,0) * dx + transform(1,1) * dy + transform(1,2);
            }

            float steeredAngle = f.bearing - startBearing;

            //std::cout << "#------------------------------#" << std::endl;
            //std::cout << "steeredAngle: " << steeredAngle * 180 / M_PI << "°" << std::endl;

            //std::cout << "target angle: " << (M_PI/2 - steeredAngle) * 180 / M_PI << std::endl;
            //std::cout << "direction angle: " << (M_PI - steeredAngle) * 180 / M_PI << std::endl;

            for (vineyard::Pole_Ptr p : (*polesVector))
            {
                if (headPoleID == -1)
                {
                    // distance pole-headPole
                    float distancePH = cv::norm(cv::Point2f(p->getCentroid().x - headPole.x, p->getCentroid().y - headPole.y));

                    if (distancePH < 1.5f && f.frameID >= 300)
                    {
                        if (headPoleID == -1)
                        {
                            std::cout << "Vedo palo di testa" << std::endl;
                            headPole = p->getCentroid();
                            headPoleID = p->ID();
                        }
                        targetPoint.x = k * std::cos(M_PI/2 - steeredAngle) + headPole.x;
                        targetPoint.y = -1 * k * std::sin(M_PI/2 - steeredAngle) + headPole.y;
                        targetDirection.x =  std::cos(M_PI - steeredAngle) + targetPoint.x;
                        targetDirection.y = -1 * std::sin(M_PI - steeredAngle) + targetPoint.y;
                    }
                }
                else
                {
                    for (vineyard::Pole_Ptr p : (*polesVector))
                    {
                        if (p->ID() == headPoleID)
                        {
                            headPole = p->getCentroid();
                            break;
                        }
                    }

                    targetPoint.x = k * std::cos(M_PI/2 - steeredAngle) + headPole.x;
                    targetPoint.y = -1 * k * std::sin(M_PI/2 - steeredAngle) + headPole.y;
                    targetDirection.x =  std::cos(M_PI - steeredAngle) + targetPoint.x;
                    targetDirection.y = -1 * std::sin(M_PI - steeredAngle) + targetPoint.y;

                    if (steeredAngle >= M_PI/2)
                    {
                        lineFollower = true;
                    }
                }
            }
        }

        // Now compute the linear and angular velocities
        if (!lineFollower || !line)
        {
            float targetDirectionAngle = std::atan2(-(targetDirection.y-targetPoint.y), targetDirection.x-targetPoint.x);
            float targetAngle = std::atan2(-1 * targetPoint.y, targetPoint.x);

            float r = cv::norm(targetPoint);
            float theta = targetDirectionAngle - targetAngle;
            float sigma = targetAngle;

            float linear = UTurnMP.computeLinearVelocity(r,theta,sigma);
            float angular = UTurnMP.computeAngularVelocity(linear,r,theta,sigma);

            std::cout << targetDirectionAngle << ";" << targetAngle << ";" << r << ";" << theta << ";" << sigma << ";" << linear << ";" << angular << std::endl;
        }
#endif
        std::vector<cv::Point2f> ptVector;
        for (nav::PT pt : f.points)
        {
            ptVector.push_back(pt.scan_pt);
        }

        GUI.drawHUD(image, f.frameID);
        GUI.drawCompass(image, f.bearing);

#ifdef LINE_FOLLOWER_
        bool useLastLine = false;
        if (line)
        {
            if (line->getPolesList().size() <= minLineSize)
            {
                useLastLine = false;
            }
            else
            {
                //draw last line with tollerance
                useLastLine = true;
                GUI.drawLastLine(image, line);
            }
        }
#endif
        GUI.drawPoints(image, ptVector);
        GUI.drawPoles(image, *polesVector);

#ifdef EGOMOTION_ESTIMATION_
        GUI.drawHeadPole(image,headPole);
        GUI.drawTarget(image,targetPoint, targetDirection);
#endif

#ifdef LINE_FOLLOWER_
        if (lineFollower)
        {
            GUI.printOperation(image, "LINE FOLLOWER");
        }
        else
        {
            GUI.printOperation(image, "OTHER");
        }

        if (nearest /*&& f.frameID >= 300*/ && lineFollower)
        {
            le.extractLineFromNearestPole(polesVector, nearest, line, useLastLine);

            if (line)
            {
                GUI.drawLine(image, *polesVector, line);
                lineParams = line->getLineParameters();


                // Check the head pole distance
                float headPoleDistance = 0.0;
                int headPoleIndex = line->getPolesList().front();
                cv::Point2f headPoleCenter = (*polesVector)[headPoleIndex]->getCentroid();
                headPoleDistance = cv::norm(headPoleCenter);

                if (headPoleDistance < minHeadPoleDistance && headPoleCenter.x < 0)
                {
                    std::cout << "!!! " << f.frameID << " - LINE END REACHED - !!!" << std::endl;
                    lineFollower = false;
                }
            }
        }

        if (/*f.frameID >= 3500 &&*/ lineFollower && line)
        {
            /// Update the EKF
            cv::Mat measurement, control;
            nav::SystemState state;
            ekf.setupMeasurementMatrix(lineParams, f.bearing, measurement);
            /// TODO: set the correct input (linear and angular velocity)
            ekf.setupControlMatrix(RandN2(1,0.2),RandN2(0,0.2),control);
            ekf.estimate(control, measurement, state);

            GUI.drawState(image, state);

            //std::cout << state.dy << " - " << state.dtheta << " - " << state.dphi << std::endl;

            // Motion planners for line following
            float desiredX = fs["lineFollower"]["desiredDistance"];
            float desiredTheta = fs["lineFollower"]["desiredTheta"];
            nav::LineFollowerMP lineFollower(fs,
                                             desiredX,
                                             desiredTheta);

            float errorX = desiredX - state.dy;
            float errorTheta = desiredTheta - state.dtheta;
            float linear = lineFollower.computeLinearVelocity(errorX, errorTheta);
            float angular = lineFollower.computeAngularVelocity(linear, errorX, errorTheta);

            float giorgios_value = 0.0f;
            if (angular == 0.0 || lineFollower.kMaxOmega == 0.0)
            {
                giorgios_value = 0.0;
            }
            else
            {
                // la prima divisione è per estrarre il segno di angular
                // la seconda parte è la proporzione "max_omega : 25 = angular : value"
                giorgios_value = (std::abs(angular) / angular) * (std::abs(angular) * 25 / lineFollower.kMaxOmega);
            }

            GUI.printGiorgiosValue(image, giorgios_value);

            //std::cout << f.frameID << ";" << state.dy << ";" << state.dtheta << ";" << state.dphi << ";" << linear << ";" << angular << ";" << giorgios_value << ";" << std::endl;

        }
#endif
        cv::imshow("navigation gui", image);
        c = cv::waitKey(waitkey);
    }


}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
