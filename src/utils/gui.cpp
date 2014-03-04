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

gui::gui(const cv::FileStorage &fs, const std::string winName, const int waitTime)
    : win_name_(winName), wait_time_(waitTime)
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

    refresh();
    cv::namedWindow(win_name_);
}

void gui::refresh()
{
    image_ = cv::Mat(cv::Size(width_, height_), CV_8UC3, lightGray_);
}

char gui::show()
{
    cv::imshow(win_name_, image_);
    return cv::waitKey(wait_time_);
}

void gui::drawHUD(const int frameNumber)
{
    /// Draw the scan area
    cv::circle(image_, center_, min_radius_ * factor_, black_);
    cv::circle(image_, center_, max_radius_ * factor_, black_);

    cv::Point2f pt1(floor(std::cos(max_angle_) * max_radius_ * factor_),
                    floor(std::sin(max_angle_) * max_radius_ * factor_));
    cv::Point2f pt2(floor(std::cos(min_angle_) * max_radius_ * factor_),
                    floor(std::sin(min_angle_) * max_radius_ * factor_));

    cv::line(image_, center_, center_ + pt1, black_);
    cv::line(image_, center_, center_ + pt2, black_);

    /// Draw the compass area
    cv::rectangle(image_, cv::Point2i(compass_x_, compass_y_),
                  cv::Point2i(compass_x_ + compass_width_, compass_y_ + compass_height_), black_, -1);

    cv::circle(image_, cv::Point2i(compass_x_ + compass_width_/2, compass_y_ + compass_height_/2),
               compass_radius_, yellow_, -1);

    // Nevermind these magic numbers, they are just for styling the GUI to align texts
    cv::putText(image_, "N", cv::Point2i(compass_x_ + compass_width_/2 - 5, compass_y_ - 3),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image_, "S", cv::Point2i(compass_x_ + compass_width_/2 - 5, compass_y_ + compass_height_ + 13),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image_, "W", cv::Point2i(compass_x_ - 15, compass_y_ + compass_height_/2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
    cv::putText(image_, "E", cv::Point2i(compass_x_ + compass_width_ + 1, compass_y_ + compass_height_/2 + 5),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);

    cv::putText(image_, "FRAME: " + std::to_string(frameNumber), cv::Point2i(5,15),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);

}

void gui::drawCompass(const float angle)
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

    cv::fillPoly(image_, pptUpper, npt, 1, red_);
    cv::fillPoly(image_, pptLower, npt, 1, white_);
}

void gui::drawPoints(const std::vector<cv::Point2f> &pointsVector)
{
    for (auto p : pointsVector)
    {
        cv::line(image_, center_+factor_*p, center_+factor_*p+cv::Point2f(1,1), black_);
    }
}

void gui::drawPoleID(const vineyard::Pole::Ptr &pole)
{
    cv::putText(image_, std::to_string(pole->ID()),
                (pole->getCentroid() * factor_) + center_ + pole_id_offset_, cv::FONT_HERSHEY_SIMPLEX,
                font_scale_, darkGray_);
}

void gui::drawPolePoints(const vineyard::Pole::Ptr &pole)
{
    for (auto p : (*pole->getPointsVector()))
    {
        cv::line(image_, center_+factor_*p, center_+factor_*p+cv::Point2f(1,1), black_);
    }
}
void gui::drawPole(const vineyard::Pole::Ptr &pole,
                   const cv::Scalar &color)
{
    cv::circle(image_, (pole->getCentroid() * factor_) + center_,
               radius_, color, thickness_);
}

void gui::drawPoles(const std::vector<vineyard::Pole::Ptr> &polesVector)
{
    for (auto p : polesVector)
    {
        //std::cout << p->getStatus() << std::endl;
        if (p->getStatus() == vineyard::Pole::LOST_TRACK)
        {
            drawPole(p, red_);
            drawPoleID(p);
        }
        else if (p->getStatus() == vineyard::Pole::VALID)
        {
            drawPolePoints(p);
            drawPole(p, darkGray_);
            drawPoleID(p);
        }
    }
}

void gui::drawLine(const std::vector<vineyard::Pole::Ptr> &polesVector,
                   const vineyard::Line::Ptr &line)
{
    int k = 0;
    const std::list<vineyard::PoleIndex> list = line->getPolesList();
    int size = list.size();
    for (vineyard::PoleIndex IDX : list)
    {
        if (0 == k)
        {
            drawPole(polesVector[IDX], blue_);
            drawPoleID(polesVector[IDX]);
        }
        else if (k == (size - 1))
        {
            drawPole(polesVector[IDX], yellow_);
            drawPoleID(polesVector[IDX]);
        }
        else
        {
            drawPole(polesVector[IDX], green_);
            drawPoleID(polesVector[IDX]);
        }

        k++;
    }

    cv::Point2f pta,ptb;
    line->computeLineExtremes(pta,ptb);
    cv::line(image_, pta*factor_+center_, ptb*factor_+center_, red_, 2);
}


void gui::drawLastLine(const vineyard::Line::Ptr &line)
{
    cv::Point2f pta,ptb;
    line->computeLineExtremes2(pta,ptb);
    cv::line(image_, pta*factor_+center_, ptb*factor_+center_, lightGreen_, factor_*line_tollerance_);
}

void gui::drawState(const SystemState &state)
{
    // draw the distance with a circle
    cv::circle(image_, center_, std::abs(state.dy) * factor_, blue_);

    // draw the angle with a line
    cv::Point2f ptA(0,0), ptB;
    float d = state.dy * factor_;
    ptB.x = d * std::cos(state.dtheta);
    ptB.y = d * std::sin(state.dtheta);
    cv::line(image_, center_ + ptA, center_ + ptB, blue_);
    cv::line(image_, center_ + ptA, center_ + cv::Point2f(d,0), black_);

}

void gui::printGiorgiosValue(const float value)
{
    cv::putText(image_, "GIORGIO'S VALUE: " + std::to_string(value), cv::Point2i(5,35),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, yellow_);
}

void gui::printOperation(const std::string operation)
{
    cv::putText(image_, "OPERATION: " + operation, cv::Point2i(5,65),
                cv::FONT_HERSHEY_SIMPLEX, font_scale_, red_);
}

void gui::drawHeadPole(const cv::Point2f &headPole)
{
    cv::circle(image_, (headPole * factor_) + center_,
               radius_, green_, thickness_);
}

void gui::drawTarget(const cv::Point2f &targetPoint,
                     const cv::Point2f &targetDirection)
{
    cv::circle(image_, (targetPoint * factor_) + center_,
               radius_, red_, thickness_);

    cv::line(image_, (targetPoint * factor_) + center_, (targetDirection * factor_) + center_, red_, 2);
}

}
