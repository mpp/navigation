#include <iostream>

#include <opencv2/opencv.hpp>

#include <deque>

#include "sensors_drivers/logparser.h"

#include "data_manipulation/poleextractor.h"
#include "data_manipulation/lineextractor.h"

#include "operations/motionoperation.h"

#include "utils/gui.h"

//#define LINE_FOLLOWER_
//#define EGOMOTION_ESTIMATION_

void help();
void checkValueAndEnqueue(deque<float> &queue, const float value);

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

    std::cout << std::fixed << std::setprecision(6);
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


    int waitkey;
    fs["logparser"]["waitkey"] >> waitkey;
    int minLineSize;
    fs["lineExtractor"]["minLineSize"] >> minLineSize;
    float minHeadPoleDistance;
    fs["lineFollower"]["minHeadPoleDistance"] >> minHeadPoleDistance;

    /// Setup the GUI
    std::shared_ptr<nav::gui> GUI = std::make_shared<nav::gui>(fs, "Navigation GUI", waitkey);
    char c = 0; // waitkey char

    std::string operation;
    fs["logparser"]["operation"] >> operation;

    std::shared_ptr<nav::MotionOperation> mo;
    bool initialized = false;
    if (operation.compare("001L") == 0)
    {
        mo = std::make_shared<nav::LineFollowerMO>(fs, false, GUI);
    }
    else if (operation.compare("001R") == 0)
    {
        mo = std::make_shared<nav::LineFollowerMO>(fs, true, GUI);
    }
    else if (operation.compare("003L") == 0)
    {
        mo = std::make_shared<nav::TurnWithCompassMO>(fs, false, GUI);
    }
    else if (operation.compare("003R") == 0)
    {
        mo = std::make_shared<nav::TurnWithCompassMO>(fs, true, GUI);
    }

    nav::Control control = {0.0f, 0.0f};

    std::vector<nav::Frame> framesVector;

    nav::parseFile(fs, framesVector);

    vineyard::PoleExtractor pe(fs);
    std::shared_ptr< std::vector< vineyard::Pole::Ptr > > polesVector;

    for (nav::Frame f : framesVector)
    {
        GUI->refresh();

        GUI->drawHUD(f.frameID);
        GUI->drawCompass(f.bearing);

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

        std::vector<cv::Point2f> ptVector;
        for (nav::PT pt : f.points)
        {
            ptVector.push_back(pt.scan_pt);
        }

        GUI->drawPoints(ptVector);
        GUI->drawPoles(*polesVector);

        if ((operation.compare("001L") == 0 || operation.compare("001R") == 0) && f.frameID >= 400)
        {
            std::shared_ptr<nav::LineFollowerMO> lfmo = std::static_pointer_cast<nav::LineFollowerMO>(mo);

            lfmo->updateParameters(polesVector,
                                   control,
                                   f.bearing);

            control = lfmo->computeOperationControl();

            if (lfmo->checkOperationEnd())
            {
                cv::waitKey();
                return 0;
            }
        }
        if ((operation.compare("003L") == 0 || operation.compare("003R") == 0) && f.frameID >= 5900)
        {
            std::shared_ptr<nav::TurnWithCompassMO> twcmo = std::static_pointer_cast<nav::TurnWithCompassMO>(mo);

            if (!initialized)
            {
                if (operation.compare("003L") == 0)
                {
                    twcmo->initialize(polesVector,
                                      f.bearing,
                                      cv::Point2f(-2.5f, -2.0f));
                }
                else
                {
                    twcmo->initialize(polesVector,
                                      f.bearing,
                                      cv::Point2f(-2.5f, 2.0f));
                }
                initialized = true;
            }
            else
            {
                twcmo->updateParameters(polesVector,
                                       control,
                                       f.bearing);

                control = twcmo->computeOperationControl();

                if (twcmo->checkOperationEnd())
                {
                    cv::waitKey();
                    return 0;
                }
            }
        }

        c = GUI->show();
    }

    /**
     * TODO

        linearVelocity = UTurnMP.computeLinearVelocity(r,theta,sigma);
        angularVelocity = UTurnMP.computeAngularVelocity(linearVelocity,r,theta,sigma);

        float robotWidth = 0.6;
        float wheelRadius = 0.35;

        float lv = (linearVelocity + robotWidth * angularVelocity) / wheelRadius;
        float rv = (linearVelocity - robotWidth * angularVelocity) / wheelRadius;

        lv = lv > 0.05 ? lv : 0.05;
        rv = rv > 0.05 ? rv : 0.05;

        float maxWheelVelocityValue = (maxV + robotWidth * maxV * M_PI / 4) / wheelRadius;

        lv = lv / maxWheelVelocityValue;
        rv = rv / maxWheelVelocityValue;

        checkValueAndEnqueue(leftVel, lv);
        checkValueAndEnqueue(rightVel, rv);
        */

}

void checkValueAndEnqueue(deque<float> &queue, const float value)
{
    // buffer size
    int k = 5;
    int size = queue.size();
    if (size == 0)
    {
        queue.push_back(value);
        return;
    }

    // check if the value is in range with the others
    float min = std::numeric_limits<float>::max(),
          max = -std::numeric_limits<float>::max(),
          average = 0.0f;
    for (float v : queue)
    {
        average = average + v;
        if (v < min)
        {
            min = v;
        }
        if (v > max)
        {
            max = v;
        }
    }
    average = average / size;

    float epsilon = 0.15f;
    float variance = std::abs(value - average);

    if (variance <= epsilon)
    {
        queue.push_back(value);

        if (queue.size() >= k)
        {
            queue.pop_front();
        }
    }
}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
