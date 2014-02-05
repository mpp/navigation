#include <iostream>

#include <opencv2/opencv.hpp>

#include "data_manipulation/poleextractor.h"
#include "data_manipulation/lineextractor.h"

#include "sensors_drivers/logparser.h"
#include "utils/gui.h"

void help();

int main(int argc, char **argv)
{
    std::cout << "Hi" << std::endl;

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

    std::cout << std::fixed << std::setprecision(6) << "Hello!" << std::endl;
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

    char c = 0;
    //float angle = 0;
    //float angleIncrement = M_PI / 8;

    std::vector<nav::Frame> framesVector;

    nav::parseFile(fs, framesVector);

    std::cout << framesVector.size() << std::endl;

    vineyard::PoleExtractor pe(fs);
    vineyard::LineExtractor le(fs);
    std::shared_ptr< std::vector< vineyard::Pole::Ptr > > polesVector;

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

        GUI.drawHUD(image);
        GUI.drawCompass(image, f.bearing);
        GUI.drawPoints(image, ptVector);
        GUI.drawPoles(image, *polesVector);

        if (nearest)
        {
            vineyard::Line::Ptr line;
            le.extractLineFromNearestPole(polesVector, nearest, line);
            GUI.drawLine(image, *polesVector, line);
        }

        cv::imshow("navigation gui", image);
        c = cv::waitKey(33);
    }


}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
