/**
 * \file lineextractor.cpp
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

#include "lineextractor.h"

namespace vineyard
{

LineExtractor::LineExtractor(const cv::FileStorage &fs)
{
    maximum_pole_distance_ = fs["lineExtractor"]["maxPoleDistance"];
}

void LineExtractor::extractLineFromNearestPole(const std::shared_ptr< const std::vector< Pole::Ptr > > polesVector,
                                               const Pole::Ptr &nearest,
                                               Line::Ptr &line)
{
    LineParams lineParam;
    // Initialize a temporary Line container
    Line temp(polesVector, lineParam);

    // Initialize a list of poles' indices
    PoleIndex
            currentNearestNeighborIndex;
    std::list<int>
            indices(polesVector->size(), 0);

    for (int i = 0; i < polesVector->size(); i++)
    {
        if ((*polesVector)[i]->ID() != nearest->ID())
        {
            indices.push_back(i);
        }
        else
        {
            currentNearestNeighborIndex = i;
        }
    }

    temp.insert_head(currentNearestNeighborIndex);

    // I need to extract only the forward line
    // so I recursively search forward for the nearest neighbor the current pole
    Pole::Ptr
            currentNearestNeighbor = nearest;
    cv::Point2f
            currentCenter = currentNearestNeighbor->getCentroid();

    // Initialize a vector of point for the line fitting algorithm
    std::vector<cv::Point2f>
            linePoints;
    linePoints.push_back(currentCenter);

    for (;;)
    {
        float minSDistance = std::numeric_limits<float>::max();
        Pole::Ptr newNearestNeighbor;
        bool found = false;
        for (auto i : indices)
        {
            // search for the forward nearest neighbor
            if ((*polesVector)[i]->getCentroid().x > currentCenter.x)
            {
                float diff1 = currentCenter.x - (*polesVector)[i]->getCentroid().x;
                float diff2 = currentCenter.y - (*polesVector)[i]->getCentroid().y;
                float currentSDistance = diff1*diff1 + diff2*diff2;

                /*if (currentSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    found = true;
                    newNearestNeighbor = (*polesVector)[i];
                    currentNearestNeighborIndex = i;
                    break;
                }*/

                if (currentSDistance < minSDistance && currentSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    found = true;
                    minSDistance = currentSDistance;
                    newNearestNeighbor = (*polesVector)[i];
                    currentNearestNeighborIndex = i;
                }
            }
        }
        if (found == true)
        {
            currentNearestNeighbor = newNearestNeighbor;
            currentCenter = currentNearestNeighbor->getCentroid();
            found = false;

            // remove this pole from the list
            indices.remove(currentNearestNeighborIndex);

            // push the pole's center in the vector of line's points
            linePoints.push_back(currentCenter);

            // push the pole in the temporary container
            temp.insert_head(currentNearestNeighborIndex);
        }
        else
        {
            break;
        }
    }

    // Fit the line and set the parameters in the line object
    cv::Vec4f lineParamVec;
    cv::fitLine(linePoints, lineParamVec, CV_DIST_FAIR, 0, 0.01, 0.01);


    lineParam.head_pole_ID = currentNearestNeighbor->ID();
    lineParam.head_pole_x = currentNearestNeighbor->getCentroid().x;
    lineParam.head_pole_y = currentNearestNeighbor->getCentroid().y;
    lineParam.vx = lineParamVec[0];
    lineParam.vy = lineParamVec[1];
    lineParam.x0 = lineParamVec[2];
    lineParam.y0 = lineParamVec[3];

    temp.setLineParameters(lineParam);

    line = std::make_shared<Line>(temp);
}

/*void LineExtractor::extractLineFromNearestPole(const std::shared_ptr< const std::vector<std::shared_ptr<Pole> > > polesVector,
                                               const std::shared_ptr<Pole> &nearest,
                                               Line &line)
{

    // Initialize a list of poles' indices
    PoleIndex
            actualNearestNeighborIndex;
    std::list<int>
            indices(polesVector->size(), 0);

    for (int i = 0; i < polesVector->size(); i++)
    {
        if ((*polesVector)[i]->ID() != nearest->ID())
        {
            indices.push_back(i);
        }
        else
        {
            actualNearestNeighborIndex = i;
        }
    }

    // Initialize the line
    line.insert_head(actualNearestNeighborIndex);

    //std::cout << "Right line: " << nearest->ID();

    // I need to extract only the forward line
    // so I recursively search forward for the nearest neighbor the actual pole
    std::shared_ptr<Pole>
            actualNearestNeighbor = nearest;
    cv::Point2d
            actualCenter = actualNearestNeighbor->getCentroid();

    // Initialize a vector of point for the line fitting algorithm
    std::vector<cv::Point2f>
            linePoints;
    linePoints.push_back(actualCenter);
    for (;;)
    {
        double minSDistance = std::numeric_limits<double>::max();
        std::shared_ptr<Pole> newNearestNeighbor;
        bool found = false;
        for (auto i : indices)
        {
            // search for the forward nearest neighbor
            if ((*polesVector)[i]->getCentroid().x > actualCenter.x)
            {
                double diff1 = actualCenter.x - (*polesVector)[i]->getCentroid().x;
                double diff2 = actualCenter.y - (*polesVector)[i]->getCentroid().y;
                double actualSDistance = diff1*diff1 + diff2*diff2;

                if (actualSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    found = true;
                    newNearestNeighbor = (*polesVector)[i];
                    actualNearestNeighborIndex = i;
                    break;
                }

                if (actualSDistance < minSDistance && actualSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    found = true;
                    minSDistance = actualSDistance;
                    newNearestNeighbor = (*polesVector)[i];
                    actualNearestNeighborIndex = i;
                }
            }
        }
        if (found == true)
        {
            actualNearestNeighbor = newNearestNeighbor;
            actualCenter = actualNearestNeighbor->getCentroid();
            found = false;
            // remove this pole from the list
            indices.remove(actualNearestNeighborIndex);
            // update the line with the new pole
            line.insert_head(actualNearestNeighborIndex);
            // push the pole's center in the vector of line's points
            linePoints.push_back(actualCenter);

            //std::cout << " - " << actualNearestNeighbor->ID();
        }
        else
        {
            break;
        }
    }
    //std::cout << std::endl;

    // Fit the line and set the parameters in the line object
    /// TODO: mind the trunkation from double to float...
    cv::Vec4f lineParams;
    cv::fitLine(linePoints, lineParams, CV_DIST_FAIR, 0, 0.01, 0.01);
    line.setLineParameters(lineParams);

    //std::cout << lineParams << std::endl;
}*/

/*void LineExtractor::extractLines(const std::vector< std::shared_ptr<Pole> > &polesVector,
                                 std::vector<std::shared_ptr<Line> > &linesVector)
{
    /// Setup the poles's cloud O(n)
    pcl::PointCloud<pcl::PointXYZ>::Ptr
            polesCloud(new pcl::PointCloud<pcl::PointXYZ>());

    for (auto p : polesVector)
    {
        pcl::PointXYZ
                pt(p->getCentroid().x, p->getCentroid().y, 0.0);

        polesCloud->push_back(pt);
    }
    polesCloud->width = (int) polesCloud->points.size ();
    polesCloud->height = 1;

    /// Clusterize it O(nlogn)
    pcl::EuclideanClusterExtraction<pcl::PointXYZ>
            ec;
    std::vector<pcl::PointIndices>
            clusterIndices;

    ec.setClusterTolerance (maximum_pole_distance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (max_cluster_size_);
    ec.setSearchMethod (search_tree_);
    ec.setInputCloud (polesCloud);
    ec.extract (clusterIndices);

    for (auto indices : clusterIndices)
    {
        std::cout << "Cluster: ";
        for (auto i : indices.indices)
        {
            std::cout << polesVector[i]->ID() << " - ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    /// Check lines inliers and outliers with RANSAC O(T(C1+C2n)) T:iterations Cx:constant
    // input: cloud and indices of cluster's poles,
    // output: indices of line's poles (inliers)
    std::vector<pcl::PointIndices>
            linesIndices;
    std::vector<double>
            aVector,
            dVector;

    /// TODO: remove magic numbers -> move to a config file
    ransacLine2D(polesCloud,
                 clusterIndices,
                 0.1, 50, 0.8, 3,
                 linesIndices,
                 aVector,
                 dVector);

    /// For each line extract the head and the tail.
    //  Then add the whole vector in the output list


    // Debug check:
    if (linesIndices.size() != aVector.size() || aVector.size() != dVector.size())
    {
        std::cerr << "Bad vectors sizes!!" << std::endl;
    }

    for (int i = 0; i < linesIndices.size(); i++)
    {
        // search for the extremes
        int
                headIndex = -1,     // the index of the head in the line
                tailIndex = -1;     // the index of the tail in the line

        pcl::PointXYZ
                headPoint(-kLargeNumber, -kLargeNumber, 0.0),          // head and tail
                tailPoint(kLargeNumber, kLargeNumber, 0.0);

        // Initialize the line
        std::shared_ptr<Line> l(new Line(std::shared_ptr<const std::vector<std::shared_ptr<Pole> > >(
                   std::make_shared<const std::vector<std::shared_ptr<Pole> > >(polesVector)), aVector[i], dVector[i]));

        for (auto IDX : linesIndices[i].indices)
        {
            // If on top/right put on head
            if (polesCloud->points[IDX].x > headPoint.x || polesCloud->points[IDX].y > headPoint.y)
            {
                headIndex = IDX;
                headPoint = polesCloud->points[IDX];
                l->insert_head(IDX);
            }
            // If on bottom/left put on tail
            else if (polesCloud->points[IDX].x < tailPoint.x || polesCloud->points[IDX].y < tailPoint.y)
            {
                tailIndex = IDX;
                tailPoint = polesCloud->points[IDX];
                l->insert_tail(IDX);
            }
            // Else put as second element
            else
            {
                l->insert(IDX);
            }
        }

        linesVector.push_back(l);
    }

    //std::cout << "DEBUG" << std::endl;
}*/

} // namespace vineyard
