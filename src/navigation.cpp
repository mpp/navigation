#include <iostream>

#include <opencv2/opencv.hpp>

#include "sensors_drivers/logparser.h"

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

    std::vector<nav::Frame> framesVector;

    nav::parseFile(fs, framesVector);

    std::cout << framesVector.size() << std::endl;
}

void help()
{
    std::cout << "Usage: navigation -s <configuration file>" << std::endl;
    exit(-1);
}
