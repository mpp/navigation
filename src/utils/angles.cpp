#include "angles.h"

namespace nav {

float normalizeAngle_PI(float angle)
{
    angle = angle > M_PI ? angle - 2 * M_PI : angle;
    angle = angle <= -M_PI ? angle + 2 * M_PI : angle;

    return angle;
}

}   // namespace nav
