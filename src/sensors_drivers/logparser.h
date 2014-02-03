#ifndef LOGPARSER_H
#define LOGPARSER_H

#include <fstream>
#include <locale>
#include <iomanip>

#include <opencv2/opencv.hpp>

#include <boost/tokenizer.hpp>

#include "../utils/commontypes.h"

namespace nav
{

/*!
 * \brief parseFile It populate the vector with the scanned frames
 * \param [in] file the filename+path of the log file
 * \param [out] logFrames the vector of frames
 */
void parseFile(const cv::FileStorage &fs,
               std::vector<Frame> &logFrames);

}

#endif // LOGPARSER_H
