#include <iostream>

#include <opencv2/opencv.hpp>

#include "sensors_drivers/logparser.h"

#include "data_manipulation/poleextractor.h"
#include "data_manipulation/lineextractor.h"

#include "data_manipulation/ekfstateestimator.h"

#include "motion_planners/linefollowermp.h"

#include "utils/gui.h"

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
    vineyard::LineExtractor le(fs);
    std::shared_ptr< std::vector< vineyard::Pole::Ptr > > polesVector;
    vineyard::LineParams lineParams;

    nav::EKFStateEstimator ekf;

    vineyard::Line::Ptr line;

    bool lineFollower = true;

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

        vineyard::Pole::Ptr nearest;
        pe.findNearestPole(*polesVector, false, nearest);

        std::vector<cv::Point2f> ptVector;
        for (nav::PT pt : f.points)
        {
            ptVector.push_back(pt.scan_pt);
        }

        GUI.drawHUD(image, f.frameID);
        GUI.drawCompass(image, f.bearing);

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
        GUI.drawPoints(image, ptVector);
        GUI.drawPoles(image, *polesVector);

        if (lineFollower)
        {
            GUI.printOperation(image, "LINE FOLLOWER");
        }
        else
        {
            GUI.printOperation(image, "OTHER");
        }

        if (nearest && f.frameID >= 3500 && lineFollower)
        {
            le.extractLineFromNearestPole(polesVector, nearest, line, useLastLine);
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

        if (f.frameID >= 3500 && lineFollower)
        {
            /// Update the EKF
            cv::Mat measurement, control;
            nav::SystemState state;
            ekf.setupMeasurementMatrix(lineParams, f.bearing, measurement);
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
        cv::imshow("navigation gui", image);
        c = cv::waitKey(waitkey);
    }


}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
