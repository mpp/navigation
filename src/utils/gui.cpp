#include "gui.h"

namespace nav
{

cv::Scalar read(const cv::FileNode &node)
{
    std::vector<int> temp;
    node >> temp;
    cv::Scalar ret(temp[0],temp[1],temp[2],temp[3]);

    return ret;
}

gui::gui(const cv::FileStorage &fs)
{
    width_      = fs["gui"]["wWidth"];
    height_     = fs["gui"]["wHeight"];
    factor_     = fs["gui"]["wFactor"]; // 1mt = factor pixels
    font_scale_ = fs["gui"]["wFontScale"];

    min_radius_ = fs["laserscanfilter"]["minDistance"];
    max_radius_ = fs["laserscanfilter"]["maxDistance"];
    min_angle_  = fs["laserscanfilter"]["minAngle"];
    max_angle_  = fs["laserscanfilter"]["maxAngle"];

    radius_     = fs["gui"]["poleRadius"];
    thickness_  = fs["gui"]["lineThickness"];

    std::vector<int> center;
    fs["gui"]["center"] >> center;
    center_.x = center[0];
    center_.y = center[1];

    line_tollerance_ = fs["lineExtractor"]["maxDistanceFromLastLine"];

    red_        = read(fs["gui"]["colors"]["red"]);
    darkGray_   = read(fs["gui"]["colors"]["darkGray"]);
    white_      = read(fs["gui"]["colors"]["white"]);
    black_      = read(fs["gui"]["colors"]["black"]);
    green_      = read(fs["gui"]["colors"]["green"]);
    lightGreen_  = read(fs["gui"]["colors"]["lightGreen"]);
    lightGray_  = read(fs["gui"]["colors"]["lightGray"]);
    blue_       = read(fs["gui"]["colors"]["blue"]);
    yellow_     = read(fs["gui"]["colors"]["yellow"]);

    std::vector<int> poleIDOffset;
    fs["gui"]["poleIDOffset"] >> poleIDOffset;
    pole_id_offset_.x = poleIDOffset[0];
    pole_id_offset_.y = poleIDOffset[1];

    std::vector<int> compassWHXY;
    fs["gui"]["compassWHXY"] >> compassWHXY;

    compass_width_  = compassWHXY[0];
    compass_height_ = compassWHXY[1];
    compass_x_      = compassWHXY[2];
    compass_y_      = compassWHXY[3];
    compass_radius_ = fs["gui"]["compassRadius"];
}

void gui::drawHUD(cv::Mat &image, const int frameNumber)
{
    /// Draw the scan area
    image = cv::Mat(cv::Size(width_, height_), CV_8UC3, lightGray_);

    cv::circle(image, center_, min_radius_ * factor_, black_);
    cv::circle(image, center_, max_radius_ * factor_, black_);

    cv::Point2f pt1(floor(std::cos(max_angle_) * max_radius_ * factor_),
                    floor(std::sin(max_angle_) * max_radius_ * factor_));
    cv::Point2f pt2(floor(std::cos(min_angle_) * max_radius_ * factor_),
                    floor(std::sin(min_angle_) * max_radius_ * factor_));

    cv::line(image, center_, center_ + pt1, black_);
    cv::line(image, center_, center_ + pt2, black_);

    /// Draw the compass area
    cv::rectangle(image, cv::Point2i(compass_x_, compass_y_),
                  cv::Point2i(compass_x_ + compass_width_, compass_y_ + compass_height_), black_, -1);

    cv::circle(image, cv::Point2i(compass_x_ + compass_width_/2, compass_y_ + compass_height_/2),
               compass_radius_, yellow_, -1);

    // Nevermind these magic numbers, they are just for styling the GUI to align texts
    cv::putText(image, "N", cv::Point2i(compass_x_ + compass_width_/2 - 5, compass_y_ - 3),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image, "S", cv::Point2i(compass_x_ + compass_width_/2 - 5, compass_y_ + compass_height_ + 13),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image, "W", cv::Point2i(compass_x_ - 15, compass_y_ + compass_height_/2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image, "E", cv::Point2i(compass_x_ + compass_width_ + 1, compass_y_ + compass_height_/2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);

    cv::putText(image, "FRAME: " + std::to_string(frameNumber), cv::Point2i(5,15),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);

}

void gui::drawCompass(cv::Mat &image, const float angle)
{
    const float normalizedAngle = angle;
    cv::Point2i compassCenter(compass_x_ + compass_width_/2, compass_y_ + compass_height_/2);

    cv::Point2i north(std::cos(normalizedAngle)*compass_radius_,
                      -std::sin(normalizedAngle)*compass_radius_);

    cv::Point2i south(-north.x, -north.y);
    cv::Point2i east(std::cos(normalizedAngle-M_PI/2)*15,
                    -std::sin(normalizedAngle-M_PI/2)*15);
    cv::Point2i west(-east.x, -east.y);

    cv::Point2i points[2][3];

    points[0][0] = north + compassCenter;
    points[0][1] = east + compassCenter;
    points[0][2] = west + compassCenter;

    points[1][0] = south + compassCenter;
    points[1][1] = east + compassCenter;
    points[1][2] = west + compassCenter;

    int npt[] = { 3 };
    const cv::Point2i * pptUpper[1] = { points[0] };
    const cv::Point2i * pptLower[1] = { points[1] };

    cv::fillPoly(image, pptUpper, npt, 1, red_);
    cv::fillPoly(image, pptLower, npt, 1, white_);
}

void gui::drawPoints(cv::Mat &image,
                     const std::vector<cv::Point2f> &pointsVector)
{
    for (auto p : pointsVector)
    {
        cv::line(image, center_+factor_*p, center_+factor_*p+cv::Point2f(1,1), black_);
    }
}

void gui::drawPoleID(cv::Mat &image,
                     const vineyard::Pole::Ptr &pole)
{
    cv::putText(image, std::to_string(pole->ID()),
                (pole->getCentroid() * factor_) + center_ + pole_id_offset_, cv::FONT_HERSHEY_SIMPLEX,
                font_scale_, darkGray_);
}

void gui::drawPolePoints(cv::Mat &image,
                         const vineyard::Pole::Ptr &pole)
{
    for (auto p : (*pole->getPointsVector()))
    {
        cv::line(image, center_+factor_*p, center_+factor_*p+cv::Point2f(1,1), black_);
    }
}
void gui::drawPole(cv::Mat &image,
                   const vineyard::Pole::Ptr &pole,
                   const cv::Scalar &color)
{
    cv::circle(image, (pole->getCentroid() * factor_) + center_,
               radius_, color, thickness_);
}

void gui::drawPoles(cv::Mat &image,
                    const std::vector<vineyard::Pole::Ptr> &polesVector)
{
    for (auto p : polesVector)
    {
        //std::cout << p->getStatus() << std::endl;
        if (p->getStatus() == vineyard::Pole::LOST_TRACK)
        {
            drawPole(image, p, red_);
            drawPoleID(image, p);
        }
        else if (p->getStatus() == vineyard::Pole::VALID)
        {
            drawPolePoints(image, p);
            drawPole(image, p, darkGray_);
            drawPoleID(image, p);
        }
    }
}

void gui::drawLine(cv::Mat &image,
                   const std::vector<vineyard::Pole::Ptr> &polesVector,
                   const vineyard::Line::Ptr &line)
{
    int k = 0;
    const std::list<vineyard::PoleIndex> list = line->getPolesList();
    int size = list.size();
    for (vineyard::PoleIndex IDX : list)
    {
        if (0 == k)
        {
            drawPole(image, polesVector[IDX], blue_);
            drawPoleID(image, polesVector[IDX]);
        }
        else if (k == (size - 1))
        {
            drawPole(image, polesVector[IDX], yellow_);
            drawPoleID(image, polesVector[IDX]);
        }
        else
        {
            drawPole(image, polesVector[IDX], green_);
            drawPoleID(image, polesVector[IDX]);
        }

        k++;
    }

    cv::Point2f pta,ptb;
    line->computeLineExtremes(pta,ptb);
    cv::line(image, pta*factor_+center_, ptb*factor_+center_, red_, 2);
}


void gui::drawLastLine(cv::Mat &image,
                       const vineyard::Line::Ptr &line)
{
    cv::Point2f pta,ptb;
    line->computeLineExtremes2(pta,ptb);
    cv::line(image, pta*factor_+center_, ptb*factor_+center_, lightGreen_, factor_*line_tollerance_);
}

void gui::drawState(cv::Mat &image,
                    const SystemState &state)
{
    // draw the distance with a circle
    cv::circle(image, center_, state.dy * factor_, blue_);

    // draw the angle with a line
    cv::Point2f ptA(0,0), ptB;
    float d = state.dy * factor_;
    ptB.x = d * std::cos(state.dtheta);
    ptB.y = d * std::sin(state.dtheta);
    cv::line(image, center_ + ptA, center_ + ptB, blue_);
    cv::line(image, center_ + ptA, center_ + cv::Point2f(d,0), black_);

}

void gui::printGiorgiosValue(cv::Mat &image,
                             const float value)
{
    cv::putText(image, "GIORGIO'S VALUE: " + std::to_string(value), cv::Point2i(5,35),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
}

}
