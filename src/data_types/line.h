/**
 * \file line.h
 * \Author: Michele Marostica
 * \brief: This class is an abstraction of a line composed of poles
 *
 * Copyright (c) 2013, Michele Marostica (michelemaro AT gmail DOT com)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1 - Redistributions of source code must retain the above copyright notice,
 *            this list of conditions and the following disclaimer.
 * 2 - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LINE_H
#define LINE_H

#include <list>
#include <memory>

#include "pole.h"

namespace vineyard {

typedef struct LineParams_
{
    double
        vx,             // (vx,vy) is a vector parallel to the line
        vy,
        x0,             // (x0,y0) is a point on the line
        y0;

    int
        head_pole_ID;   // The ID of the head pole of the line

    double
        head_pole_x,    // Position of the head pole
        head_pole_y;

} LineParams;

class Line
{

public:

    /** I need methods to:
     *  - get the head and tail
     *  - get start and stop point for line drawing
     *  + track the line
     *      - line ID
     *      - how to track a line??
     *      - track for ghost poles...
     *  - set/get the line color
     */

    Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > polesVector);

    Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > polesVector,
         double a,
         double d);

    Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > polesVector,
         const cv::Vec4d &lineParams);

    ~Line();

    void setLineParameters(const cv::Vec4d &lineParameters);

    void insert(PoleIndex idx);

    void insert_head(PoleIndex idx);

    void insert_tail(PoleIndex idx);

    int ID() const { return id_; }

    cv::Scalar getColor() const { return color_; }

    double a() const { return a_; }
    double d() const { return d_; }

    const std::shared_ptr< std::list<PoleIndex> > getPolesList() const;

    void computeLineExtremes(cv::Point2d &a, cv::Point2d &b) const;

// private methods
private:

// private data
private:

    int
        id_;

    std::shared_ptr< std::list<PoleIndex> >
        pole_list_;             //< indices of the pole array the head and the tail of the list are the extremes of the line

    std::shared_ptr< const std::vector< std::shared_ptr<Pole> > >
        poles_vector_;        //< a reference to the poles vector

    double
        a_,                     //< the slope of the line -> y = ax + d
        d_;                     //< the distance from the origin -> y = ax + d

    cv::Vec4d
        line_parameters_;       // (vx, vy, x0, y0) where: (vx, vy) is a vector colinear to the line and (x0, y0) is a point on the line

    //DEBUG VARIABLES
    cv::Scalar color_;

    static int progressive_id_;
};

} // namespace vineyard

#endif // LINE_H
