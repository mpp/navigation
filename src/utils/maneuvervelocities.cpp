#include <utils/maneuvervelocities.h>

namespace nav {

ManeuverVelocities::ManeuverVelocities(cv::FileStorage &fs)
{
    cv::FileNode velocities = fs["maxVelocities"];

    for (cv::FileNode node : velocities)
    {
        std::string maneuverName = node.name();
        float maneuverMaxVel = (*(node.begin()));

        maneuver_max_velocity_map_[maneuverName] = maneuverMaxVel;

        //std::cout << "name: " << maneuverName << " vel: " << maneuverMaxVel << std::endl;
    }
}

float ManeuverVelocities::getMaxVel(std::string maneuver)
{
    return maneuver_max_velocity_map_.find(maneuver)->second;
}

}
