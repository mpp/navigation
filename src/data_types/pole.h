/**
 * \file pole.h
 * \Author: Michele Marostica
 * \brief: This class contains information of a pole
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

#ifndef POLE_H
#define POLE_H

#include <deque>
#include <memory>

#include <opencv2/opencv.hpp>

namespace vineyard
{

typedef int PoleIndex;

class Pole;
typedef std::shared_ptr<Pole> Pole_Ptr;
typedef std::shared_ptr<const Pole> Pole_ConstPtr;

class Pole
{
// Public methods
public:

    typedef std::shared_ptr<Pole> Ptr;
    typedef std::shared_ptr<Pole const> ConstPtr;

    enum TYPE {
        VALID = 0,
        JUST_SCANNED = -1,  //!< a pole just scanned, you can't know if it is new or a track of an old one
        LOST_TRACK = -2,    //!< a pole with a lost track
        ASSOCIATED = -3,
        DUMMY = 98,
        ERROR = -99         //!< error pole
    };

    /*!
     * \brief Pole
     * \param pointsVector
     */
    Pole(const std::shared_ptr< const std::vector<cv::Point2f> > &pointsVector);

    /*!
     * \brief Pole create a dummy pole
     * \param x
     * \param y
     */
    Pole(const int id, const float x, const float y);

    ~Pole();

    /*!
     * \brief requestID request a unique ID for the pole
     */
    void requestID();

    /*!
     * \brief getCentroid
     * \return the centroid
     */
    cv::Point2f getCentroid() const;

    /*!
     * \brief getPointsVector
     * \param [out] pointsVector a copy of the points vector
     */
    const std::shared_ptr< const std::vector<cv::Point2f> > & getPointsVector() const;

    /*!
     * \brief ID
     * \return the pole's identifier
     */
    int ID() const;

    /*!
     * \brief getStatus
     * \return the pole's status
     */
    int getStatus() const;

    /*!
     * \brief setStatus
     */
    void setStatus(TYPE status);

    /*!
     * \brief updateCentroid updates the centroid using the kalman filter
     * \param [in] newCentroid the measurement correction update
     */
    void updateCentroid(const cv::Point2f &newCentroid);

    void updateCentroid(const cv::Point2f &newCentroid, const cv::Matx22f &control);

    /*!
     * \brief updatePointsVector
     * \param newPointsVector
     */
    void updatePointsVector(const std::shared_ptr< const std::vector<cv::Point2f> > &newPointsVector);

    /*!
     * \brief idSort sorting function by ASC ID
     * \param a
     * \param b
     * \return
     */
    static inline bool idSort(const Pole::ConstPtr a, const Pole::ConstPtr b) { return idSort(*a, *b); }
    static inline bool idSort(const Pole a, const Pole b) { return a.id_ > b.id_; }

// Private methods
private:

    /*!
     * \brief computeCentroid it computes the centroid of the points_vector_
     */
    void computeCentroid();

    static void initializeAvailableIDs();

// Private data
private:

    int id_;                                //!< a pole identifier

    int status_;                            //!< the pole status, see the enumeration

    int freshness_;                         //!< a value that decrease each time that the pole is not
                                            //!< seen in a frame. When it reach 0, the pole become
                                            //!< invalid.

    cv::Point2f centroid_;                  //!< the estimated pole centroid

    std::shared_ptr< const std::vector<cv::Point2f> >
        points_vector_;                     //!< the set of points in the pole's cluster

    // KALMAN Filter parameters
    cv::KalmanFilter kf_pole_;

    cv::Mat_<float> kf_state_;

    cv::Mat kf_process_noise_;

    cv::Mat_<float> kf_measurement_;

    std::vector<cv::Point2f> kf_history_;   //!< estimation history

    static std::deque<int> available_ids_;

};

} // namespace vineyard
#endif // POLE_H
