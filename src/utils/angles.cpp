#include <utils/angles.h>

namespace nav {

float normalizeAngle_PI(float angle)
{
    angle = angle > M_PI ? angle - 2 * M_PI : angle;
    angle = angle <= -M_PI ? angle + 2 * M_PI : angle;

    return angle;
}

float normalizeAngle_2PI(float angle)
{
    angle = angle > 2 * M_PI ? angle - 2 * M_PI : angle;
    angle = angle <= 0.0f ? angle + 2 * M_PI : angle;

    return angle;
}

}   // namespace nav
