#ifndef MANEUVERVELOCITIES_H
#define MANEUVERVELOCITIES_H

#include <map>
#include <opencv2/opencv.hpp>

namespace nav {

class ManeuverVelocities
{
public:
    ManeuverVelocities(cv::FileStorage &fs);

    float getMaxVel(std::string maneuver);

private:
    std::map<std::string, float> maneuver_max_velocity_map_;
};

}

#endif // MANEUVERVELOCITIES_H
