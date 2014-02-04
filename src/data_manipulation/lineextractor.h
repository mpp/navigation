/**
 * \file lineextractor.h
 * \Author: Michele Marostica
 * \brief: This class take as input a point a vector of sparse poles and search for lines
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

#ifndef LINEEXTRACTOR_H
#define LINEEXTRACTOR_H

#include <list>
#include <memory>

#include "../data_types/line.h"

namespace vineyard
{

class LineExtractor
{
public:
    /*!
     * \brief LineExtractor
     * \param maxPoleDistance
     */
    LineExtractor(const cv::FileStorage &fs);

    /*!
     * \brief extractLineFromNearestPole
     * \param [in] polesVector
     * \param [in] nearest
     * \param [out] line
     */
    void extractLineFromNearestPole(const std::shared_ptr< const std::vector< Pole::Ptr > > polesVector,
                                    const Pole::Ptr &nearest,
                                    Line::Ptr &line);

// private methods
private:

// private data
private:

    double
        maximum_pole_distance_;
    unsigned int
        min_cluster_size_;          //< minimum number of poles in a line cluster
    unsigned int
        max_cluster_size_;          //< maximum number of poles in a line cluster
};

} // namespace vineyard

#endif // LINEEXTRACTOR_H
