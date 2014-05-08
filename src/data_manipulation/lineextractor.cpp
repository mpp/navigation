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

#include <data_manipulation/lineextractor.h>

namespace vineyard
{

LineExtractor::LineExtractor(const cv::FileStorage &fs)
    : maximum_pole_distance_(fs["lineExtractor"]["maxPoleDistance"]),
      reps_(fs["lineExtractor"]["reps"]),
      aeps_(fs["lineExtractor"]["aeps"]),
      max_distance_from_last_line_(fs["lineExtractor"]["maxDistanceFromLastLine"]),
      min_line_size_(fs["lineExtractor"]["minLineSize"])
{
}

float LineExtractor::distanceLinePole(const LineParams &lineParam,
                                      const cv::Point2f &poleCenter)
{
    float k1,k2,x,y,dx,dy;
    k1 = (poleCenter.x - lineParam.x0) / lineParam.vx;
    k2 = (poleCenter.y - lineParam.y0) / lineParam.vy;
    y = lineParam.y0 + k1*lineParam.vy;
    x = lineParam.x0 + k2*lineParam.vx;

    dx = std::abs(poleCenter.x - x);
    dy = std::abs(poleCenter.y - y);
    return (dx < dy) ? dx : dy;
}

void LineExtractor::extractAccessPathFromNearestPole(
		const std::shared_ptr< const std::vector< Pole::Ptr > > polesVector,
    const Pole::Ptr &nearest,
    Line::Ptr &line,
    const bool useLastLine)
{
    if (!nearest || !polesVector)
    {
        return;
    }

    LineParams lineParam;
    // Initialize a temporary Line container
    Line temp(polesVector, lineParam);

    // If I already have a line I use it as a reference to accept or refuse poles
    if (useLastLine)
    {
        lineParam = line->getLineParameters();
    }

    Pole::Ptr
        lowestButNearest = 0,
        actualPole;

    PoleIndex
        nearestIndex,
        lowestButNearestIndex;

    for (int i = 0; i < polesVector->size(); i++)
	{
        actualPole = (*polesVector)[i];
		if (actualPole->ID() != nearest->ID())
		{
            if (actualPole->getCentroid().y < 0  &&
                    (!lowestButNearest || actualPole->getCentroid().y > lowestButNearest->getCentroid().y))
			{
                if (actualPole->getCentroid().x > (1.5 + nearest->getCentroid().x))
                {
                    lowestButNearest = actualPole;
                    lowestButNearestIndex = i;
                }
			}
		}
		else
		{
			nearestIndex = i;
		}
	}

    if (!lowestButNearest)
    {
        return;
    }

    temp.insert_head(nearestIndex);
    temp.insert_head(lowestButNearestIndex);

    std::vector<cv::Point2f>
    linePoints;
    linePoints.push_back(nearest->getCentroid());
    linePoints.push_back(lowestButNearest->getCentroid());

    // Fit the line and set the parameters in the line object
	 cv::Vec4f lineParamVec;
	 cv::fitLine(linePoints, lineParamVec, CV_DIST_L1, 0, reps_, aeps_);

	 lineParam.head_pole_ID = nearest->ID();
	 lineParam.head_pole_x = nearest->getCentroid().x;
	 lineParam.head_pole_y = nearest->getCentroid().y;
	 lineParam.vx = lineParamVec[0];
	 lineParam.vy = lineParamVec[1];
	 lineParam.x0 = lineParamVec[2];
	 lineParam.y0 = lineParamVec[3];

	 temp.setLineParameters(lineParam);

	 line = std::make_shared<Line>(temp);


}

void LineExtractor::extractLineFromNearestPole(const std::shared_ptr< const std::vector< Pole::Ptr > > polesVector,
                                               const Pole::Ptr &nearest,
                                               Line::Ptr &line,
                                               const bool useLastLine)
{
    if (!nearest || !polesVector)
    {
        return;
    }

    LineParams lineParam;
    // Initialize a temporary Line container
    Line temp(polesVector, lineParam);

    // If I already have a line I use it as a reference to accept or refuse poles
    if (useLastLine)
    {
        lineParam = line->getLineParameters();
    }

    // Initialize a list of poles' indices
    PoleIndex
            currentNearestNeighborIndex;
    std::list<int>
            indices;

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

    //std::cout << std::endl;

    // Try to do the same for backward poles
    for (;;)
    {
        float minSDistance = std::numeric_limits<float>::max();
        Pole::Ptr newNearestNeighbor;
        bool found = false;
        float nearestLineDistance = 0.0f;
        for (auto i : indices)
        {
            // search for the forward nearest neighbor
            if ((*polesVector)[i]->getCentroid().x < currentCenter.x)
            {
                float diff1 = currentCenter.x - (*polesVector)[i]->getCentroid().x;
                float diff2 = currentCenter.y - (*polesVector)[i]->getCentroid().y;
                float currentSDistance = diff1*diff1 + diff2*diff2;

                if (currentSDistance < minSDistance && currentSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    // If I already have a line I use it as a reference to accept or refuse poles
                    float d = 0.0f;
                    if (useLastLine)
                    {
                        // Compute the distance from the pole to the line
                        d = distanceLinePole(lineParam, (*polesVector)[i]->getCentroid());
                        //std::cout << (*polesVector)[i]->ID() << ":" << d << " - ";
                    }
                    // If there is not a line, d will be 0.0f
                    if (d <= max_distance_from_last_line_)
                    {
                        found = true;
                        nearestLineDistance = d;
                        minSDistance = currentSDistance;
                        newNearestNeighbor = (*polesVector)[i];
                        currentNearestNeighborIndex = i;
                    }
                }
            }
        }
        if (found == true)
        {
            //std::cout << "Pole in line:" << (*polesVector)[currentNearestNeighborIndex]->ID() << std::endl;
            currentNearestNeighbor = newNearestNeighbor;
            currentCenter = currentNearestNeighbor->getCentroid();
            found = false;

            // remove this pole from the list
            indices.remove(currentNearestNeighborIndex);

            // push the pole's center in the vector of line's points
            //linePoints.push_back(currentCenter);
            for (cv::Point2f p : (*polesVector)[currentNearestNeighborIndex]->getPoints())
            {
                linePoints.push_back(p);
            }

            // push the pole in the temporary container
            temp.insert_head(currentNearestNeighborIndex);
        }
        else
        {
            //std::cout << std::endl;
            break;
        }
    }


    currentNearestNeighbor = nearest;
    currentCenter = currentNearestNeighbor->getCentroid();

    for (;;)
    {
        float minSDistance = std::numeric_limits<float>::max();
        Pole::Ptr newNearestNeighbor;
        bool found = false;
        float nearestLineDistance = 0.0f;
        for (auto i : indices)
        {
            // search for the forward nearest neighbor
            if ((*polesVector)[i]->getCentroid().x > currentCenter.x)
            {
                float diff1 = currentCenter.x - (*polesVector)[i]->getCentroid().x;
                float diff2 = currentCenter.y - (*polesVector)[i]->getCentroid().y;
                float currentSDistance = diff1*diff1 + diff2*diff2;

                int id = (*polesVector)[i]->ID();

                if (currentSDistance < minSDistance && currentSDistance < maximum_pole_distance_*maximum_pole_distance_)
                {
                    // If I already have a line I use it as a reference to accept or refuse poles
                    float d = 0.0f;
                    if (useLastLine)
                    {
                        // Compute the distance from the pole to the line
                        d = distanceLinePole(lineParam, (*polesVector)[i]->getCentroid());
                        //std::cout << (*polesVector)[i]->ID() << ":" << d << " - ";
                    }
                    // If there is not a line, d will be 0.0f
                    if (d <= max_distance_from_last_line_)
                    {
                        found = true;
                        nearestLineDistance = d;
                        minSDistance = currentSDistance;
                        newNearestNeighbor = (*polesVector)[i];
                        currentNearestNeighborIndex = i;
                    }
                }
            }
        }
        if (found == true)
        {
            //std::cout << "Pole in line:" << (*polesVector)[currentNearestNeighborIndex]->ID() << std::endl;
            currentNearestNeighbor = newNearestNeighbor;
            currentCenter = currentNearestNeighbor->getCentroid();
            found = false;

            // remove this pole from the list
            indices.remove(currentNearestNeighborIndex);

            // push the pole's center in the vector of line's points
            //linePoints.push_back(currentCenter);
            for (cv::Point2f p : (*polesVector)[currentNearestNeighborIndex]->getPoints())
            {
                linePoints.push_back(p);
            }

            // push the pole in the temporary container
            temp.insert_head(currentNearestNeighborIndex);
        }
        else
        {
            //std::cout << std::endl;
            break;
        }

    }

    if (temp.getPolesList().size() < min_line_size_)
    {
        lineParam.head_pole_ID = currentNearestNeighbor->ID();
        lineParam.head_pole_x = currentNearestNeighbor->getCentroid().x;
        lineParam.head_pole_y = currentNearestNeighbor->getCentroid().y;
        temp.setLineParameters(lineParam);
        line = std::make_shared<Line>(temp);
        //line.reset();
        return;
    }
    // Fit the line and set the parameters in the line object
    cv::Vec4f lineParamVec;
    cv::fitLine(linePoints, lineParamVec, CV_DIST_L1, 0, reps_, aeps_);

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

} // namespace vineyard
