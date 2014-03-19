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
float runningAverage(std::deque<float> &queue, const float value);

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

    std::deque<float> leftVel, rightVel;
    float maxV;
    fs["globalMP"]["maxV"] >> maxV;


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

        if (f.oper_t.compare("003R") != 0 && f.oper_t.compare("003L") != 0 &&
            f.oper_t.compare("001R") != 0 && f.oper_t.compare("001L") != 0)
        {
            continue;
        }

        if ((operation.compare("001L") == 0 || operation.compare("001R") == 0))
        {
            std::shared_ptr<nav::LineFollowerMO> lfmo = std::static_pointer_cast<nav::LineFollowerMO>(mo);

            lfmo->updateParameters(polesVector,
                                   control,
                                   f.bearing,
                                   96*M_PI/180);

            //std::cout << "bearing " << f.bearing << std::endl;

            control = lfmo->computeOperationControl();

            if (lfmo->checkOperationEnd())
            {
                cv::waitKey();
                return 0;
            }
        }
        if ((operation.compare("003L") == 0 || operation.compare("003R") == 0) /*&& f.frameID > 2400*/)
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
                                      cv::Point2f(-2.5f, 1.5f));
                }
                initialized = true;
            }
            else
            {
                twcmo->updateParameters(polesVector,
                                       control,
                                       f.bearing,
                                       96*M_PI/180);

                control = twcmo->computeOperationControl();

                /** LOG */
                float r, theta, sigma;
                twcmo->getLogStatus(r, theta, sigma);
                /**     */
                if (twcmo->checkOperationEnd())
                {
                    cv::waitKey();
                    return 0;
                }
            }
        }

        float robotWidth = 0.6;
        float wheelRadius = 0.35;

        float lv = (control.linear + robotWidth * control.angular) / wheelRadius;
        float rv = (control.linear - robotWidth * control.angular) / wheelRadius;

        //lv = lv > 0.05 ? lv : 0.05;
        //rv = rv > 0.05 ? rv : 0.05;

        float maxWheelVelocityValue = (maxV + robotWidth * maxV * M_PI / 4) / wheelRadius;

        lv = lv / maxWheelVelocityValue;
        rv = rv / maxWheelVelocityValue;

        //float leftAveraged = runningAverage(leftVel, lv);
        //float rightAveraged = runningAverage(rightVel, rv);

        //std::cout << "(lv,rv) = (" << lv << ", " << rv << ")" << std::endl;
        std::cout << lv << ";" << rv << std::endl;
        c = GUI->show();
    }
}

float runningAverage(std::deque<float> &queue, const float value)
{
    // buffer size
    int k = 16;
    int size = queue.size();
    if (size == 0)
    {
        queue.push_back(value);
        return value;
    }
    queue.push_back(value);

    if (queue.size() >= k)
    {
        queue.pop_front();
    }

    size = queue.size();
    // check if the value is in range with the others
    float average = 0.0f;
    float totalWeight = 0.0f;
    int counter = 1;
    for (float v : queue)
    {
        average = average + v * (float)counter / size;
        totalWeight = totalWeight + (float)counter / size;
        counter++;
    }
    average = average / totalWeight;

    return average;
}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
