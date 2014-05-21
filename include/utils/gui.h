#ifndef GUI_H
#define GUI_H

#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>

#include <data_types/commontypes.h>
#include <data_manipulation/ekfstateestimator.h>

namespace nav
{

class gui
{
public:
    gui(const cv::FileStorage &fs, const std::string winName, const int waitTime);

    char show();
    char show(const int ms);

    void refresh();

    void drawHUD(const int frameNumber);

    void drawCompass(const float angle);

    void drawPoints(const std::vector<cv::Point2f> &pointsVector);

    void drawPolePoints(const vineyard::Pole::Ptr &pole);

    void drawPoleID(const vineyard::Pole::Ptr &pole);

    void drawPole(const vineyard::Pole::Ptr &pole,
                  const cv::Scalar &color);

    void drawPoles(const std::vector< vineyard::Pole::Ptr > &polesVector);

    void drawCross(const cv::Point2f &pt);

    void drawLine(const std::vector< vineyard::Pole::Ptr > &polesVector,
                  const vineyard::Line::Ptr &line,
                  const float desiredDistance = -1);

    void drawLastLine(const vineyard::Line::Ptr &line, bool safe = true);

    void drawIdealDistanceLine(const float desiredDistance);

    void drawState(const nav::SystemState &state);

    void printOperation(const std::string operation);

    void printGiorgiosValue(const float value);

    void drawHeadPole(const cv::Point2f &headPole);

    void drawTarget(const cv::Point2f &targetPoint,
                    const cv::Point2f &targetDirection);

    void drawPixelPath(const cv::Point2f &inPt);

    void drawObstacle(bool isObstacle,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                      std::shared_ptr< std::vector<pcl::PointIndices> > &clusterIndices,
                      float minRange,
                      float maxRange,
                      float minAngle,
                      float maxAngle);
private:

    cv::Mat image_;
    std::string win_name_;
    int wait_time_;

    int width_;
    int height_;
    int factor_; // 1mt = factor pixels
    double font_scale_;

    cv::Point2f offset_;

    float min_radius_;
    float max_radius_;
    float min_angle_;
    float max_angle_;

    cv::Point2f pole_id_offset_;
    int radius_;
    int thickness_;
    cv::Point2f center_;

    int compass_width_;
    int compass_height_;
    int compass_x_;
    int compass_y_;
    int compass_radius_;

    float line_tollerance_;

    cv::Scalar red_;
    cv::Scalar darkGray_;
    cv::Scalar white_;
    cv::Scalar black_;
    cv::Scalar green_;
    cv::Scalar lightGreen_;
    cv::Scalar lightRed_;
    cv::Scalar gray_;
    cv::Scalar lightGray_;
    cv::Scalar blue_;
    cv::Scalar yellow_;
};

} // namespace nav

#endif // GUI_H
