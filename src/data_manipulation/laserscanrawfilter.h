#ifndef LASERSCANRAWFILTER_H
#define LASERSCANRAWFILTER_H

#include <opencv2/opencv.hpp>

#include "../utils/commontypes.h"

namespace nav {

class LaserScanRawFilter
{

public:
    LaserScanRawFilter(const cv::FileStorage &fs);

    std::vector<PT> filter(std::vector<PT> &toFilter);

private:
    LaserScanRawFilter();

private:
    float min_distance_;
    float max_distance_;
    float min_angle_;
    float max_angle_;
};

} // namespace nav

#endif // LASERSCANRAWFILTER_H
