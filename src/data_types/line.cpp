/**
 * \file line.cpp
 * \Author: Michele Marostica
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

#include "line.h"

namespace vineyard {
/*
Line::Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > polesVector)
{
    poles_vector_ = std::move(polesVector);
    a_ = 0.0;
    d_ = 0.0;

    pole_list_ = std::shared_ptr< std::list<PoleIndex> >(new std::list<PoleIndex>());
}

Line::Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > polesVector, double a, double d)
{
    poles_vector_ = std::move(polesVector);
    a_ = a;
    d_ = d;

    /// TODO: convert to vector form

    pole_list_ = std::shared_ptr< std::list<PoleIndex> >(new std::list<PoleIndex>());
}
*/
Line::Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > > &polesVector,
           const cv::Vec4d &lineParams)
{
    poles_vector_ = polesVector;

    setLineParameters(lineParams);

    pole_list_.reset();
}

void Line::setLineParameters(const cv::Vec4d &lineParameters)
{
    line_parameters_ = lineParameters;

    // lineParams = (vx, vy, x0, y0)
    // vector form to y = ax + d form:
    // a = vy/vx
    // d = y0 - a*x0
    /// check by 0 division
    if (0.0 == lineParameters[0])
    {
        a_ = std::numeric_limits<double>::max();
    }
    else
    {
        a_ = lineParameters[1]/lineParameters[0];
    }
    d_ = lineParameters[3] - a_ * lineParameters[2];
}

Line::~Line()
{
    poles_vector_.reset();
    //poles_vector_.~__shared_ptr();
    pole_list_.~__shared_ptr();
}

void Line::insert(PoleIndex idx)
{
    insert_tail(idx);
}

void Line::insert_head(PoleIndex idx)
{
    pole_list_->push_front(idx);
}

void Line::insert_tail(PoleIndex idx)
{
    pole_list_->push_back(idx);
}

const std::shared_ptr< const std::list<PoleIndex> > & Line::getPolesList() const
{
    return pole_list_;
}

void Line::computeLineExtremes(cv::Point2d &a, cv::Point2d &b) const
{
    double
            x = 0.0,
            y = 0.0,
            dx = 0.0,
            dy = 0.0;
    cv::Point2d polePt;

    // head point
    polePt = (*poles_vector_)[pole_list_->front()]->getCentroid();
    y = a_ * polePt.x + d_;
    x = (polePt.y - d_) / a_;

    dx = std::abs(polePt.x - x);
    dy = std::abs(polePt.y - y);

    // project the point in x and in y on the line, take the nearer
    if (dx < dy)
    {
        a.x = x;
        a.y = polePt.y;
    }
    else
    {
        a.x = polePt.x;
        a.y = y;
    }

    // tail point
    polePt = (*poles_vector_)[pole_list_->back()]->getCentroid();
    y = a_ * polePt.x + d_;
    x = (polePt.y - d_) / a_;

    dx = std::abs(polePt.x - x);
    dy = std::abs(polePt.y - y);

    // project the point in x and in y on the line, take the nearer
    if (dx < dy)
    {
        b.x = x;
        b.y = polePt.y;
    }
    else
    {
        b.x = polePt.x;
        b.y = y;
    }
}

} // namespace vineyard
