#ifndef GUI_H
#define GUI_H

#include <opencv2/opencv.hpp>

namespace nav
{

class gui
{
public:
    gui(const cv::FileStorage &fs);
/*
    void drawHUD(cv::Mat &image);

    void drawPolePoints(cv::Mat &image,
                        const std::shared_ptr<vineyard::Pole> &pole);

    void drawPoleID(cv::Mat &image,
                    const std::shared_ptr<vineyard::Pole> &pole);

    void drawPole(cv::Mat &image,
                  const std::shared_ptr<vineyard::Pole> &pole,
                  const cv::Scalar &color);

    void drawPoles(cv::Mat &image,
                   const std::vector< std::shared_ptr<vineyard::Pole> > &polesVector);

    void drawLine(cv::Mat &image,
                  const std::vector< std::shared_ptr<vineyard::Pole> > &polesVector,
                  const vineyard::Line &line);
    */
private:

    int width_;
    int height_;
    int factor_; // 1mt = factor pixels
    double font_scale_;

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
