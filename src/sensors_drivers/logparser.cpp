#include "logparser.h"

namespace nav
{

void parseFile(const cv::FileStorage &fs,
               std::vector<Frame> &logFrames)
{
    /// Open the input log file
    std::string path = fs["logparser"]["path"];
    std::string filename = fs["logparser"]["file"];
    std::ifstream ifsLog((path + filename).c_str());

    if (!ifsLog)
    {
        std::cerr << "Cannot open file: " << filename << std::endl;
        exit(-1);
    }

    /// Set the correct locale
    std::cout << std::fixed << std::setprecision(6) << std::endl;
    setlocale(LC_NUMERIC, "C");

    /// Setup temp variables
    std::string temp; // placeholder for useless data
    std::string epoch;
    int frameID = -1;
    int i = 0;
    float dist = 0.0;
    float beardeg = 0.0;
    std::string oper_t = "";

    int lastFrame = -1;
    Frame currentFrame;
    PT currentPT;

    /// Remember to ignore the first header line
    ifsLog >> temp; // tokens are separated by ; not by spaces!!

    /// Set the right token separator
    boost::char_separator<char> sep(";");

    // 540 measures from -135° to 135°
    float const accuracy = 270.0/540.0;
    float const toRadian = M_PI/180.0;

    while (!ifsLog.eof())
    {
        /// Take a row from the csv file and separate it into tokens
        ifsLog >> temp; //>> fucking_space;
        boost::tokenizer< boost::char_separator<char> > tokens(temp, sep);
        boost::tokenizer< boost::char_separator<char> >::iterator tk_it = tokens.begin();

        /// Take data from tokens
        // Each row has the following structure:
        // Epoch;frame;i;dist1;rssi;gyroh;beardeg;road;place;oper;s3m
        epoch   = *tk_it;                 // epoch
        tk_it++;
        frameID = std::stoi(*tk_it);      // frameID
        tk_it++;
        i       = std::stoi(*tk_it);      // i
        tk_it++;
        dist = 0.001 * std::stof(*tk_it); // dist
        tk_it++;
        tk_it++;                          // rssi
        tk_it++;                          // gyroh
        beardeg = std::stof(*tk_it);      // beardeg
        tk_it++;
        tk_it++;                          // road
        tk_it++;                          // place
        oper_t  = *tk_it;                 // oper
        tk_it++;                          // s3m

        /// Setup the point
        // 540 measures from -135° to + 135°
        currentPT.angle = (accuracy * i - 135.0) * toRadian;
        currentPT.scan_pt.x = dist * std::cos(currentPT.angle);
        currentPT.scan_pt.y = -1 * dist * std::sin(currentPT.angle);
        currentPT.ray = dist;

        /// Filter input data
        float
                minAngle = fs["laserscanfilter"]["minAngle"],
                maxAngle = fs["laserscanfilter"]["maxAngle"],
                minDistance = fs["laserscanfilter"]["minDistance"],
                maxDistance = fs["laserscanfilter"]["maxDistance"];

        if (currentPT.angle < minAngle || currentPT.angle > maxAngle ||
            dist < minDistance || dist > maxDistance)
        {
            continue;
        }

        if (frameID > lastFrame)
        {
            /// Initial hack
            // If a new frame add the previous to the frames vector
            if (lastFrame != -1)
            {
                logFrames.push_back(currentFrame);
            }

            lastFrame = frameID;

            /// Initialize the current frame with new data
            currentFrame.epoch = epoch;
            currentFrame.frameID = frameID;
            currentFrame.oper_t = oper_t;
            currentFrame.bearing = beardeg*toRadian;
            // Remember to clear the points vector
            currentFrame.points.clear();
        }
        /// Add the point to the points vector
        currentFrame.points.push_back(currentPT);
    }

    // Don't forget to add the last frame!!
    if (frameID > -1)
    {
        logFrames.push_back(currentFrame);
    }
}

} // namespace nav
