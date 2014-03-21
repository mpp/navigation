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

#include "pole.h"

namespace vineyard
{

std::deque<int> Pole::available_ids_;

Pole::Pole(const std::shared_ptr< const std::vector<cv::Point2f> > &pointsVector)
{
    initializeAvailableIDs();

    status_ = VALID;
    id_ = -1;// = available_ids_.top();
    //available_ids_.pop();

    //std::cout << available_ids_.top() << std::endl;

    freshness_ = 5;

    centroid_.x = 0;
    centroid_.y = 0;

    points_vector_ = pointsVector;

    // Add points and compute the centroid
    computeCentroid();

    // Setup KALMAN
    kf_pole_.init(4,2,2);
    kf_state_ = cv::Mat(4, 1, CV_32F);
    kf_process_noise_ = cv::Mat(4, 1, CV_32FC1);
    kf_measurement_ = cv::Mat::zeros(2, 1, CV_32F);

    kf_state_ << centroid_.x,centroid_.y,0,0;


    kf_pole_.statePost.at<float>(0) = centroid_.x;
    kf_pole_.statePost.at<float>(1) = centroid_.y;
    kf_pole_.statePost.at<float>(2) = 0;
    kf_pole_.statePost.at<float>(3) = 0;


    kf_pole_.transitionMatrix = *(cv::Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);

    // Given that I have no control matrix, I have to rely more on the measurement.
    // This means to have a low measureNoiseCovariance and an high processNoiseCovariance.
    cv::setIdentity(kf_pole_.measurementMatrix);
    cv::setIdentity(kf_pole_.processNoiseCov, cv::Scalar::all(0.01));
    cv::setIdentity(kf_pole_.measurementNoiseCov, cv::Scalar::all(1e-5));
    cv::setIdentity(kf_pole_.errorCovPost, cv::Scalar::all(0.1));

    kf_pole_.gain *(cv::Mat_<float>(4, 2) << 0.9,0.9,0.9,0.9,0.9,0.9,0.9,0.9);

    kf_history_.clear();
}

Pole::Pole(const int id, const float x, const float y)
{
    status_ = DUMMY;
    id_ = id;
    centroid_.x = x;
    centroid_.y = y;
    points_vector_ = nullptr;
}

Pole::~Pole()
{
    if (id_ != -1)
    {
        available_ids_.push_back(id_);
    }
}

void Pole::requestID()
{
    id_ = available_ids_.front();
    available_ids_.pop_front();
}

cv::Point2f Pole::getCentroid() const
{
    return centroid_;
}

void Pole::initializeAvailableIDs()
{
    if (available_ids_.size() == 0)
    {
        for (int i = 999; i >= 0; i--)
        {
            available_ids_.push_front(i);
        }
    }
}

const std::shared_ptr< const std::vector<cv::Point2f> > & Pole::getPointsVector() const
{
    return points_vector_;
}

int Pole::ID() const
{
    return id_;
}

int Pole::getStatus() const
{
    return status_;
}

void Pole::setStatus(TYPE status)
{
    if (status == LOST_TRACK)
    {
        freshness_ = freshness_ - 1;
        //std::cout << "FRESHNESS:\t" << freshness_ << std::endl;

        if (freshness_ < 0)
        {
            status_ = LOST_TRACK;
        }
    }
    else
    {
        status_ = status;
    }
}


void Pole::updateCentroid(const cv::Point2f &newCentroid)
{
    // Do the prediction update of the Kalman filter
    cv::Mat predicted = kf_pole_.predict();

    // Use newCentroid to correct the Kalman filter prediction
    kf_measurement_.at<float>(0) = newCentroid.x;
    kf_measurement_.at<float>(1) = newCentroid.y;

    //std::cout << std::endl << "-------" << std::endl << "ID:\t" << id_ << std::endl;
    //std::cout << "STATE:\t" << kf_pole_.statePost << std::endl;
    //std::cout << "PREDICTED:\t" << predicted << std::endl;
    //std::cout << "MEASURED:\t" << kf_measurement_ <<std::endl;

    cv::Mat estimated = kf_pole_.correct(kf_measurement_);

    kf_state_ = estimated;

    //std::cout << "ESTIMATED:\t" << estimated << std::endl;

    centroid_ = cv::Point2f(estimated.at<float>(0),estimated.at<float>(1));
}

void Pole::updatePointsVector(const std::shared_ptr< const std::vector<cv::Point2f> > &newPointsVector)
{
    points_vector_.reset();
    points_vector_ = newPointsVector;
}

void Pole::computeCentroid()
{
    std::vector<cv::Point2f>::const_iterator it = points_vector_->begin();

    // Update the centroid within the points vector
    double value = 1.0 / points_vector_->size();

    centroid_.x = 0;
    centroid_.y = 0;

    for (; it != points_vector_->end(); it++)
    {
        centroid_.x = centroid_.x + (*it).x * value;
        centroid_.y = centroid_.y + (*it).y * value;
    }
}

std::vector<cv::Point2f> Pole::getPoints() const
{
    return *points_vector_;
}

} // namespace vineyard
