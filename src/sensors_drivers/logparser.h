#ifndef LOGPARSER_H
#define LOGPARSER_H

#include <fstream>
#include <locale>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <boost/tokenizer.hpp>

namespace nav
{

typedef struct PT_ {

  cv::Point2f scan_pt;      //!< scan point coordinates (cartesian)
  float ray;                //!< scan point coordinates (polar)
  float angle;              //!< scan point coordinates (polar)

} PT;

typedef struct Frame_ {

  int frameID;              //!< The ID assigned to the frame
  std::string epoch;        //!< The epoch of the frame
  std::string oper_t;       //!< The operation of the frame TODO: put the operations in a enumeration
  int bearing;              //!< The bearing of the robot in the frame
  std::vector<PT_> points;  //!< The vector of scanned points in this frame

} Frame;

/*!
 * \brief parseFile It populate the vector with the scanned frames
 * \param [in] file the filename+path of the log file
 * \param [out] logFrames the vector of frames
 */
void parseFile(const std::string &file, std::vector<Frame> &logFrames);

}

#endif // LOGPARSER_H
