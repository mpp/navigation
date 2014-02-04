#ifndef GUI_H
#define GUI_H

#include <opencv2/opencv.hpp>

#include "../data_types/commontypes.h"

namespace nav
{

class gui
{
public:
    gui(const cv::FileStorage &fs);

    void drawHUD(cv::Mat &image);

    void drawCompass(cv::Mat &image,
                     const float angle);

    void drawPolePoints(cv::Mat &image,
                        vineyard::Pole::ConstPtr &pole);

    void drawPoleID(cv::Mat &image,
                    vineyard::Pole::ConstPtr &pole);

    void drawPole(cv::Mat &image,
                  vineyard::Pole::ConstPtr &pole,
                  const cv::Scalar &color);

    void drawPoles(cv::Mat &image,
                   const std::vector< vineyard::Pole::Ptr > &polesVector);

    void drawLine(cv::Mat &image,
                  const std::vector< vineyard::Pole::Ptr > &polesVector,
                  const vineyard::Line &line);

private:

    int width_;
    int height_;
    int factor_; // 1mt = factor pixels
    double font_scale_;

    float min_radius_;
    float max_radius_;
    float min_angle_;
    float max_angle_;

    cv::Point2i pole_id_offset_;
    int radius_;
    int thickness_;
    cv::Point2i center_;

    int compass_width_;
    int compass_height_;
    int compass_x_;
    int compass_y_;
    int compass_radius_;

    cv::Scalar red_;
    cv::Scalar darkGray_;
    cv::Scalar white_;
    cv::Scalar black_;
    cv::Scalar green_;
    cv::Scalar lightGray_;
    cv::Scalar blue_;
    cv::Scalar yellow_;
};

} // namespace nav

#endif // GUI_H
