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
#include <limits>

#include "pole.h"

namespace vineyard {

typedef struct LineParams_
{
    float
        vx,             //!< (vx,vy) is a vector parallel to the line
        vy,
        x0,             //!< (x0,y0) is a point on the line
        y0;

//    float               //!< Polar coordinates of the line:
//        r,              //!< absolute distance of the line from the origin
//        theta;          //!< angle (-M_PI, M_PI] of the line

    int
        head_pole_ID;   //!< The ID of the head pole of the line

    float
        head_pole_x,    //!< Position of the head pole
        head_pole_y;

} LineParams;

typedef std::shared_ptr<LineParams> LineParams_Ptr;

class Line
{

public:

    typedef std::shared_ptr<Line> Ptr;
    typedef std::shared_ptr<Line const> ConstPtr;

    /*!
     * \brief Line setup a line with a vector of poles and its parameters
     * \param polesVector
     * \param lineParams
     */
    Line(const std::shared_ptr< const std::vector< Pole::Ptr > > &polesVector,
         const LineParams &lineParams);

    ~Line();

    void setLineParameters(const LineParams &lineParameters);

    LineParams getLineParameters();

    /*!
     * \brief insert assign a pole to the line
     * \param idx
     */
    void insert(PoleIndex idx);

    /*!
     * \brief insert_head assign a pole to the line and insert its index in the head of the list
     * \param idx
     */
    void insert_head(PoleIndex idx);

    /*!
     * \brief insert_tail assign a pole to the line and insert its index in the tail of the list
     * \param idx
     */
    void insert_tail(PoleIndex idx);

    /*!
     * \brief getPolesList return a ref to the indices of the poles assigned to the line
     * \return
     */
    const std::list<PoleIndex> &getPolesList() const;

    /*!
     * \brief computeLineExtremes compute the extremes of a segment of the line for drawing purposes
     * \param a start point
     * \param b end point
     */
    void computeLineExtremes(cv::Point2f &a, cv::Point2f &b) const;
    void computeLineExtremes2(cv::Point2f &a, cv::Point2f &b) const;

// private methods
private:

// private data
private:

    std::list<PoleIndex>
        pole_list_;             //!< indices of the pole array the head and the tail of the list are the extremes of the line

    std::shared_ptr< const std::vector< Pole::Ptr > >
        poles_vector_;          //!< a reference to the poles vector

    LineParams
        line_parameters_;
};

} // namespace vineyard

#endif // LINE_H
