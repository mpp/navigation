#ifndef COMMONTYPES_H
#define COMMONTYPES_H

#include "pole.h"
#include "line.h"

namespace nav {

typedef struct PT_ {

  cv::Point2f scan_pt;      //!< scan point coordinates (cartesian)
  float ray;                //!< scan point coordinates (polar)
  float angle;              //!< scan point coordinates (polar)

} PT;

typedef struct Frame_ {

  int frameID;              //!< The ID assigned to the frame
  std::string epoch;        //!< The epoch of the frame
  std::string oper_t;       //!< The operation of the frame TODO: put the operations in a enumeration
  float bearing;            //!< The bearing of the robot in the frame (rad)
  std::vector<PT_> points;  //!< The vector of scanned points in this frame

} Frame;

} // namespace nav

#endif // COMMONTYPES_H
