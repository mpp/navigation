#include <iostream>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <deque>

#include "sensors_drivers/logparser.h"
#include "data_manipulation/poleextractor.h"
#include "data_manipulation/lineextractor.h"
#include "operations/motionoperation.h"
#include "utils/gui.h"

void help();

bool endsWith(const std::string& s, const std::string& suffix);

bool obstacleDetector(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &filteredCloud,
                      std::shared_ptr< std::vector<pcl::PointIndices> > &clusterIndices,
                      float minRange,
                      float maxRange,
                      float minAngle,
                      float maxAngle,
                      float clusterTolerance,
                      int minClusterSize);

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

    /// Setup for the operation
    std::shared_ptr<nav::MotionOperation> mo;
    bool initialized = false;
    if (operation.compare("F01L") == 0)
    {
        mo = std::make_shared<nav::LineFollowerMO>(fs, false, GUI);
    }
    else if (operation.compare("F01R") == 0)
    {
        mo = std::make_shared<nav::LineFollowerMO>(fs, true, GUI);
    }
    else if (operation.compare("H01L") == 0)
    {
        mo = std::make_shared<nav::LineFollowerMO>(fs, false, GUI);
    }
    else if (operation.compare("T01L") == 0)
    {
        mo = std::make_shared<nav::TurnWithCompassMO>(fs, false, GUI);
    }
    else if (operation.compare("T01R") == 0)
    {
        mo = std::make_shared<nav::TurnWithCompassMO>(fs, true, GUI);
    }
    else if (operation.compare("P01L") == 0
             || operation.compare("P01R") == 0
             || operation.compare("P02R") == 0
             || operation.compare("P03R") == 0)
    {
        mo = std::make_shared<nav::SpecialTargetMO>(fs, operation, GUI);
    }

    nav::Control control = {0.0f, 0.0f};

    std::vector<nav::Frame> framesVector;

    /// TODO: switch to a YAML format log?
    std::string logName,logPath;
    fs["logparser"]["file"] >> logName;
    fs["logparser"]["path"] >> logPath;
    if (endsWith(logName, ".csv"))
    {
        nav::parseFile(fs, framesVector);
    }
    else
    {
        // caso yml del frame finale dell'operazione che ha dato errore
        cv::FileStorage errorFrameLog(logPath+logName, cv::FileStorage::READ);
        nav::Frame errorFrame;
        errorFrameLog["bearing"] >> errorFrame.bearing;
        errorFrameLog["frameID"] >> errorFrame.frameID;
        errorFrameLog["epoch"] >> errorFrame.epoch;
        errorFrameLog["oper_t"] >> errorFrame.oper_t;

        std::vector<cv::Point2f> ptVec;
        errorFrameLog["points"] >> ptVec;

        for (cv::Point2f pt : ptVec)
        {
            float temp = cv::norm(pt);
            errorFrame.points.push_back({pt, temp, std::atan2(pt.y, pt.x)});
        }

        framesVector.push_back(errorFrame);
    }

    vineyard::PoleExtractor pe(fs);
    std::shared_ptr< std::vector< vineyard::Pole::Ptr > > polesVector;

    for (nav::Frame f : framesVector)
    {
        GUI->refresh();
        GUI->drawHUD(f.frameID);
        GUI->drawCompass(f.bearing);

        /// Setup the point cloud
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


        pcl::PointCloud<pcl::PointXYZ>::Ptr
                filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::shared_ptr< std::vector<pcl::PointIndices> > obstacleIndices;
        float
                minRange = 0.2f,
                maxRange = 4.0f,
                minAngle = -180*M_PI/180,
                maxAngle = 180*M_PI/180,
                clusterTolerance = 0.1,
                minClusterSize = 4;
        bool isObstacle = obstacleDetector(tempCloud,
                                           filteredCloud,
                                           obstacleIndices,
                                           minRange,
                                           maxRange,
                                           minAngle,
                                           maxAngle,
                                           clusterTolerance,
                                           minClusterSize);

        /// Search for poles
        pe.elaborateCloud(tempCloud, polesVector);

        std::vector<cv::Point2f> ptVector;
        for (nav::PT pt : f.points)
        {
            ptVector.push_back(pt.scan_pt);
        }

        GUI->drawPoints(ptVector);
        GUI->drawPoles(*polesVector);
        GUI->drawObstacle(isObstacle, filteredCloud, obstacleIndices, minRange, maxRange, minAngle, maxAngle);

        std::cout << operation << " - " << f.oper_t << std::endl;
        /// Check the operation frame
        if (f.oper_t.compare(operation) != 0/* || f.frameID <= 3000*/)
        {
            continue;
        }

        /// Select the right operation to do
        if (operation.compare("F01L") == 0 ||
            operation.compare("F01R") == 0 ||
            operation.compare("H01L") == 0)     // H01L to test the line follower if it works also for the headers follower problem
        {
            std::shared_ptr<nav::LineFollowerMO> lfmo = std::static_pointer_cast<nav::LineFollowerMO>(mo);

            if (!initialized)
            {
                float desiredDistanceFromLine = 2.3;
                lfmo->initialize(desiredDistanceFromLine);
                initialized = true;
            }

            lfmo->updateParameters(polesVector,
                                   control,
                                   f.bearing,
                                   93*M_PI/180);

            control = lfmo->computeOperationControl();

            float lineAngle;
            cv::Point2f initialPolePosition;
            lfmo->getFinalStatus(lineAngle, initialPolePosition);

            float progress = lfmo->checkOperationEnd();
            //std::cout << "progress: " << progress << " - initialPolePosition: " << initialPolePosition.x << std::endl;
            if (progress == 1)
            {
                cv::waitKey();
                return 0;
            }

            /// TODO: switch to a YAML format log?
//            if (lfmo->checkOperationEnd() == 1)
//            {

//                // messi qui solo per test -> devono essere variabili globali
//                float lineAngle;
//                cv::Point2f initialPolePosition;
//                lfmo->getFinalStatus(lineAngle, initialPolePosition);

//                /**LOG*/
//                // usa i tuoi dati, quelli globali, dove io uso quelli del frame
//                cv::FileStorage log("operation_log.yml", cv::FileStorage::WRITE);
//                log << "epoch" << f.epoch;
//                log << "frameID" << f.frameID;
//                log << "oper_t" << f.oper_t;
//                log << "bearing" << f.bearing;
//                log << "lineAngle" << lineAngle;
//                log << "headPole" << initialPolePosition;
//                log << "points" << "[";
//                for (vineyard::Pole::ConstPtr pole : (*polesVector))
//                {
//                    for (cv::Point2f pt : pole->getPoints())
//                    {
//                        log << pt;
//                    }
//                }
//                log << "]";

//                log.release();
//                /**LOG*/

//                cv::waitKey();
//                return 0;
//            }
        }
        if ((operation.compare("T01L") == 0 || operation.compare("T01R") == 0))
        {
            std::shared_ptr<nav::TurnWithCompassMO> twcmo = std::static_pointer_cast<nav::TurnWithCompassMO>(mo);

            /// Debug - initial status variables
            // messi qui solo per test -> devono essere variabili globali
            cv::Point2f initialPolePosition(-0.3,-1.62);    // da linefollower
            float lineAngle = 0.044f;                         // da linefollower
            float forwardDistance = 4.5f;   // da setup
            float fixedTurnAngle = M_PI/2;  // da setup
            float fixedTurnRadius = 1.45f;   // da setup
            float headPoleThreshold = 2.0f;   // da setup
            //
            if (!initialized)
            {
                twcmo->initialize(polesVector,
                                  f.bearing,
                                  initialPolePosition,
                                  lineAngle,
                                  forwardDistance,
                                  fixedTurnAngle,
                                  fixedTurnRadius,
                                  headPoleThreshold);
                initialized = true;
            }
            else
            {
                twcmo->updateParameters(polesVector,
                                       control,
                                       f.bearing,
                                       260*M_PI/180);

                control = twcmo->computeOperationControl();


                float progress = twcmo->checkOperationEnd();
                std::cout << "progress: " << progress << std::endl;
                if (progress == 1)
                {
                    /// Get data for log
                    // messi qui solo per test -> devono essere variabili globali
                    float r, theta, sigma;
                    cv::Point2f targetPoint, headPole;
                    float currentLineAngle = 0.0f;
                    twcmo->getLogStatus(r, theta, sigma, targetPoint, headPole, currentLineAngle);

                    /**LOG*/std::ofstream logStream("operation_log.csv", std::ios::app);
                    /**LOG*/logStream << f.epoch << ";" << f.frameID << ";" << f.oper_t << ";"
                    /**LOG*/          << r << ";" << theta << ";" << sigma << ";" << currentLineAngle << ";"
                    /**LOG*/          << targetPoint.x << ";" << targetPoint.y << ";" << headPole.x << ";" << headPole.y;
                    /**LOG*/logStream << ";yes" << std::endl;
                    /**LOG*/logStream.close();

                    cv::waitKey();
                    return 0;
                }
            }
        }
        if (operation.compare("P01L") == 0
                || operation.compare("P01R") == 0
                || operation.compare("P02R") == 0
                || operation.compare("P03R") == 0)
        {
            std::shared_ptr<nav::SpecialTargetMO> stmo = std::static_pointer_cast<nav::SpecialTargetMO>(mo);

            /// Debug - initial status variables
            // messi qui solo per test -> devono essere variabili globali
            /// Posizione del palo fisso
            // punto (x,y)
            cv::Point2f fixedPolePosition(9.5f, 4.5f);      // da setup

            /// ATTENTO, il vettore va in coordinate polari:
            // il primo valore è la distanza del target dal palo fisso (positiva e in metri),
            // il secondo è l'angolo di direzione del target rispetto al palo fisso (tra -PI e + PI in radianti)
            cv::Vec2f targetPoleVector(-2.0f, 0.0f);            // da setup

            /// questa è la direzione voluta del robot al target
            // angolo in radianti
            float targetBearing = (-6)*M_PI/180;                   // da setup

            if (!initialized)
            {
                stmo->initialize(f.bearing,
                                 fixedPolePosition,
                                 targetBearing,
                                 targetPoleVector);
                initialized = true;
            }
            else
            {
                stmo->updateParameters(polesVector,
                                       f.bearing);

                control = stmo->computeOperationControl();

                /// Get data for log
                // messi qui solo per test -> devono essere variabili globali
                float r, theta, sigma;
                cv::Point2f targetPoint, fixedPole;
                stmo->getLogStatus(r, theta, sigma, targetPoint, fixedPole);

                /// Check the progress of the current operation
                // this check is the same for each operation so it can be moved down outside this if tree
                // currently it is here for debugging purposes
                float progress = stmo->checkOperationEnd();
                std::cout << "progress: " << progress << " - r: " << r << " - diff: " << std::abs(theta - sigma) << std::endl;

                if (progress == 1)
                {
                    cv::waitKey();
                    return 0;
                }

            }
        }

        if (isObstacle)
        {
            control = {0.0f, 0.0f};
        }

        /// Robot specific operations
        /// These operations should be moved in a robot class with a different namespace
        // Compute the left and right velocities
        float robotWidth = 0.6;
        float wheelRadius = 0.35;

        float lv = (control.linear + robotWidth * control.angular) / wheelRadius;
        float rv = (control.linear - robotWidth * control.angular) / wheelRadius;

        float maxWheelVelocityValue = (maxV + robotWidth * maxV * M_PI / 4) / wheelRadius;

        lv = lv / maxWheelVelocityValue;
        rv = rv / maxWheelVelocityValue;

        std::cout << "(lin, ome) = (" << control.linear << ", " << control.angular << ")" << std::endl;
        std::cout << "(lv,  rv ) = (" << lv << ", " << rv << ")" << std::endl;

        c = GUI->show();
    }
}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}

bool endsWith(const std::string& s, const std::string& suffix)
{
    return s.rfind(suffix) == (s.size()-suffix.size());
}

bool obstacleDetector(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &filteredCloud,
                      std::shared_ptr< std::vector<pcl::PointIndices> > &clusterIndices,
                      float minRange,
                      float maxRange,
                      float minAngle,
                      float maxAngle,
                      float clusterTolerance,
                      int minClusterSize)
{
    // Filter the cloud
    filteredCloud->clear();

    float
            squaredMinRange = minRange * minRange,
            squaredMaxRange = maxRange * maxRange;
    for (pcl::PointXYZ pt : cloud->points)
    {
        float squaredRange = pt.x * pt.x + pt.y * pt.y;
        float angle = std::atan2(pt.y, pt.x);

        if (squaredRange >= squaredMinRange &&
                squaredRange <= squaredMaxRange
                &&
            angle >= minAngle &&
                angle <= maxAngle)
        {
           filteredCloud->push_back(pt);
        }
    }
    filteredCloud->width = (int) filteredCloud->points.size ();
    filteredCloud->height = 1;

    if (filteredCloud->points.size() <= 0)
        return false;

    // Search for a cluster
    std::vector<pcl::PointIndices> vector;
    clusterIndices = std::make_shared< std::vector<pcl::PointIndices> >(vector);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr
            tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (filteredCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ>
            ec;

    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (std::numeric_limits<int>::max());
    ec.setSearchMethod (tree);
    ec.setInputCloud (filteredCloud);
    ec.extract (vector);

    if (clusterIndices->size() >= 0)
        return true;
    return false;
}
