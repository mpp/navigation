#include "gui.h"

namespace nav
{

gui::gui(const cv::FileStorage &fs)
{
    width_      = fs["gui"]["wWidth"];
    height_     = fs["gui"]["wHeight"];
    factor_     = fs["gui"]["wFactor"]; // 1mt = factor pixels
    font_scale_ = fs["gui"]["wFontScale"];

    radius_     = fs["gui"]["poleRadius"];
    thickness_  = fs["gui"]["lineThickness"];
    std::vector<int> center;
    fs["gui"]["center"] >> center;
    center_.x = center[0];
    center_.y = center[1];

    red_ = (cv::Scalar) fs["gui"]["colors"]["red"];
    darkGray_ = (cv::Scalar) fs["gui"]["colors"]["darkGray"];
    white_ = (cv::Scalar) fs["gui"]["colors"]["white"];
    black_ = (cv::Scalar) fs["gui"]["colors"]["black"];
    green_ = (cv::Scalar) fs["gui"]["colors"]["green"];
    lightGray_ = (cv::Scalar) fs["gui"]["colors"]["lightGray"];
    blue_ = (cv::Scalar) fs["gui"]["colors"]["blue"];
    yellow_ = (cv::Scalar) fs["gui"]["colors"]["yellow"];

    std::vector<int> poleIDOffset;
    fs["gui"]["poleIDOffset"] >> poleIDOffset;
    pole_id_offset_.x = poleIDOffset[0];
    pole_id_offset_.y = poleIDOffset[1];

    std::vector<int> compassWHXY;
    fs["gui"]["compassWHXY"] >> compassWHXY;

    compass_width_ = compassWHXY[0];
    compass_height_ = compassWHXY[1];
    compass_x_ = compassWHXY[2];
    compass_y_ = compassWHXY[3];
    compass_radius_ = fs["gui"]["compassRadius"];
}

}
