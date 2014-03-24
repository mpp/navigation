/**
 * \file poleextractor.h
 * \Author: Michele Marostica
 * \brief: This class take as input a point cloud and return
 *         the estimated centroid of extracted poles
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

#ifndef POLEEXTRACTOR_H
#define POLEEXTRACTOR_H

#include <pcl/common/common_headers.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/opencv.hpp>

#include "../data_types/pole.h"

namespace vineyard
{

class PoleExtractor
{
// Public functions and methods
public:
    PoleExtractor(const cv::FileStorage &fs);

    /*!
//     * \brief elaborateCloud take as input a cloud, extract the poles and update the vector of
//     *        poles with new, tracked and lost-tracked poles
//     * \param source
//     * \param polesVector
//     */
    void elaborateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                        std::shared_ptr<std::vector<Pole::Ptr> > &polesVector);

//    /*!
//     * \brief elaboratePoints take as input a vector of points (scanned from the laserscanner),
//     *                        extract the polse and update the vector of poles with new, tracked
//     *                        and lost-tracked poles
//     * \param source
//     * \param polesVector
//     */
//    void elaboratePoints(const std::vector<cv::Point2f> &source,
//                         std::shared_ptr< std::vector< Pole::Ptr > > &polesVector);

    /*!
     * \brief findNearestPole
     * \param [in] polesVector the input poles vector
     * \param [in] onRight true to search on right, false to search on left
     * \param [out] nearest the nearest pole
     */
    void findNearestPole(const std::vector< Pole::Ptr > &polesVector,
                         const bool onRight,
                         std::shared_ptr<Pole> &nearest);

// Private method
private:
//    void clusterize(const std::vector< cv::Point2f > &source,
//                    std::vector< std::vector<int> > &clusterIndices);


//    void poleFromClusters(const std::vector< cv::Point2f > &source,
//                          std::vector< std::vector<int> > &clusterIndices,
//                          std::shared_ptr< std::vector< Pole::Ptr > > &polesVector) const;
    /*!
//     * @brief clusterize take as input the scanned cloud and return pole clusters
//     * @param [in] source the cloud source
//     * @param [out] clusterIndices the vector of points' indices for each cluster
//     */
    void clusterize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                    std::shared_ptr<std::vector<pcl::PointIndices> > &clusterIndices) const;

    /*!
//     * \brief polesFromClusters take as input the pole clusters and compute poles's data
//     * \param [in] source
//     * \param [in] clusterIndices
//     * \param [out] polesVector
//     */
    void polesFromClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                           const std::shared_ptr< const std::vector<pcl::PointIndices> > &clusterIndices,
                           std::shared_ptr<std::vector<std::shared_ptr<Pole> > > &polesVector) const;

    void clearNoise();
// Private data
private:

    float
        cluster_tolerance_;         //< cluster's point max distance
    int
        min_cluster_size_;          //< minimum number of point in a cluster
    int
        max_cluster_size_;          //< maximum number of point in a cluster
    float
        maximum_pole_distance_;     //< maximum distance for pole tracking

//    int branching_factor_;
//    int iterations_;

    std::shared_ptr< std::vector< Pole::Ptr > >
        actual_poles_vector_;       //< vector of actual poles
};

} // namespace vineyard

#endif // POLEEXTRACTOR_H
