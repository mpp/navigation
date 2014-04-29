#include <data_manipulation/poleextractor.h>

#include <utility>
//#include <opencv2/flann/flann_base.hpp>

namespace vineyard
{

PoleExtractor::PoleExtractor(const cv::FileStorage &fs)
{
    fs["poleExtractor"]["clusterTolerance"] >> cluster_tolerance_;
    fs["poleExtractor"]["minClusterSize"] >> min_cluster_size_;
    fs["poleExtractor"]["maxClusterSize"] >> max_cluster_size_;
    fs["poleExtractor"]["maximumPolesDistance"] >> maximum_pole_distance_;

    //fs["poleExtractor"]["branchingFactor"] >> branching_factor_;
    //fs["poleExtractor"]["iterations"] >> iterations_;

    actual_poles_vector_.reset();
    std::vector< Pole::Ptr > vector;
    actual_poles_vector_ = std::make_shared< std::vector< Pole::Ptr > >(vector);
}

/*void PoleExtractor::elaboratePoints(const std::vector<cv::Point2f> &source,
                                    std::shared_ptr< std::vector<Pole::Ptr> > &polesVector)
{

}

void PoleExtractor::clusterize(const std::vector<cv::Point2f> &source,
                               std::vector<std::vector<int> > &clusterIndices)
{
    // create the kmeans parameters structure:
    cvflann::KMeansIndexParams kmean_params(branching_factor_,
                                            iterations_,
                                            cvflann::FLANN_CENTERS_KMEANSPP);

    // create matrix of samples, this matrix you have to fill with your actual points
    //cvflann::Matrix<float> samples( pt, source.size(), 2 );
    cv::Mat1f samples( source.size(), 2 );

    for (int i = 0; i < source.size(); i++)
    {
        samples(i,0) = source[i].x;
        samples(i,1) = source[i].y;
    }
    //std::cout << "-------" << std::endl << samples << std::endl;

    cvflann::Matrix<float> samplesMatrix((float*)samples.data, samples.rows, samples.cols);

    // create matrix of centers, the specified rows will be kinda the upper limit of clusters
    // you'll get
    cv::Mat1f centers( std::floor(source.size() / 4), 2 );
    cvflann::Matrix<float> centersMatrix((float*)centers.data, centers.rows, centers.cols);

    // apply hierarchical clustering which returns the true numbers of clusters ( < centers.rows )
    int true_number_clusters = cvflann::hierarchicalClustering< cvflann::L1<float> >(samplesMatrix,
                                                                                     centersMatrix,
                                                                                     kmean_params );

    // since you get less clusters than you specified we can also truncate our matrix.
    centers = centers.rowRange(cv::Range(0,true_number_clusters));
    for (int i = 0; i < true_number_clusters; i++)
    {
        centers(i,0) = centersMatrix.data[i*centersMatrix.cols + 0];
        centers(i,1) = centersMatrix.data[i*centersMatrix.cols + 1];
    }

    std::cout << centers << std::endl << "-------" << std::endl;
}*/


void PoleExtractor::elaborateCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                   std::shared_ptr< std::vector< Pole::Ptr > > &polesVector)
{
    polesVector.reset();
    std::vector< Pole::Ptr > vector;
    polesVector = std::make_shared< std::vector< Pole::Ptr > >(vector);

    std::shared_ptr< std::vector<pcl::PointIndices> >
            clusterIndices;

    // Extract clusters
    clusterize(source,
               clusterIndices);

    std::shared_ptr< std::vector< std::shared_ptr<Pole> > >
            tempPolesVector;

    // Extract poles
    polesFromClusters(source,
                      clusterIndices,
                      tempPolesVector);

    // Update actual poles vector with new ones,
    // terminate lost-tracked poles and update tracked poles.

    // If the first call just copy the temp vector, all poles are new
    if (0 == actual_poles_vector_->size())
    {
        for (auto p : *tempPolesVector)
        {
            p->setStatus(Pole::VALID);
            polesVector->push_back(p);
            actual_poles_vector_->push_back(p);
        }
//        clearNoise();
        return;
    }

    // I have to iterate actual poles and searche in the temp the nearest neighbor within a
    // fixed ray. If no poles are found -> lost-track, else the pole must be updated.
    // Finally I have to add new poles (the unmatched) to the list.

    // I use a greedy and fast approach
    std::vector< std::shared_ptr<Pole> >::iterator
            actual_it = actual_poles_vector_->begin();

    for (; actual_it != actual_poles_vector_->end(); actual_it++)
    {
        // Ignore track-lost poles from update
        if ((*actual_it)->getStatus() == Pole::LOST_TRACK)
        {
            // the -- post operation is due to the for incrementation
            actual_poles_vector_->erase(actual_it--);
            continue;
        }

        double minDistance = 1 + maximum_pole_distance_;
        std::shared_ptr<Pole> nearestNeighbor;

        std::vector< std::shared_ptr<Pole> >::iterator
                temp_it = tempPolesVector->begin();
        for (; temp_it != tempPolesVector->end(); temp_it++)
        {
            // Check if the actual temp pole has been already ASSOCIATED or not
            if ((*temp_it)->getStatus() != Pole::ASSOCIATED)
            {
                // Compute the distance
                double dx = (*actual_it)->getCentroid().x - (*temp_it)->getCentroid().x;
                double dy = (*actual_it)->getCentroid().y - (*temp_it)->getCentroid().y;
                double distance = sqrt(dx*dx + dy*dy);

                // Check and update the nearest neighbor
                if (distance < maximum_pole_distance_ && distance < minDistance)
                {
                    minDistance = distance;
                    nearestNeighbor = (*temp_it);
                }
            }
        }

        // Check if it is a lost-track pole
        if (minDistance >= maximum_pole_distance_)
        {
            (*actual_it)->setStatus(Pole::LOST_TRACK);
            //std::cout << "POLE ID:\t" << (*actual_it)->ID() << " SET LOST_TRACK" << std::endl;
        }
        else
        {
            // Update the pole with its new position and points
            nearestNeighbor->setStatus(Pole::ASSOCIATED);

            (*actual_it)->updateCentroid(nearestNeighbor->getCentroid());
            //std::cout << "POLE ID:\t" << (*actual_it)->ID() << " N-NEIGHBOR FOUND" << std::endl;

            std::shared_ptr< const std::vector<cv::Point2f> >
                    tempPointsVector= nearestNeighbor->getPointsVector();
            (*actual_it)->updatePointsVector(tempPointsVector);
        }
    }

    // Now add the new poles
    std::vector< Pole::Ptr >::iterator
            temp_it = tempPolesVector->begin();
    for (; temp_it != tempPolesVector->end(); temp_it++)
    {
        if (Pole::JUST_SCANNED == (*temp_it)->getStatus())
        {
            (*temp_it)->setStatus(Pole::VALID);
            actual_poles_vector_->push_back((*temp_it));
        }
    }

//    clearNoise();

    // Copy all the poles to the output array
    for (auto i : *actual_poles_vector_)
    {
        polesVector->push_back(i);
    }
}

//void PoleExtractor::clearNoise()
//{
//    std::vector<Pole::Ptr> noNoiseVector;
//    for (Pole::Ptr curr : (*actual_poles_vector_))
//    {
//        int count = 0;
//        for (Pole::Ptr p : (*actual_poles_vector_))
//        {
//            float distance = cv::norm(curr->getCentroid() - p->getCentroid());
//            if (distance <= 1.0f)
//            {
//                count++;
//            }
//        }
//        std::cout << count << std::endl;
//        if (count <= 3)
//        {
//            noNoiseVector.push_back(curr);
//        }
//    }
//    actual_poles_vector_->clear();
//    for (auto i : noNoiseVector)
//    {
//        actual_poles_vector_->push_back(i);
//    }
//}

void PoleExtractor::findNearestPole(const std::vector<Pole::Ptr> &polesVector,
                                    const bool onRight,
                                    std::shared_ptr<Pole> &nearest)
{
    // I need to check only the squared distance
    double minSDistance = std::numeric_limits<double>::max();

    // Iterate al poles
    for (auto pole : polesVector)
    {
        // Check right-left
        if ((onRight == true && pole->getCentroid().y >= 0) || (onRight == false && pole->getCentroid().y <= 0))
        {
            // Check distance
            double actualSDistance = (pole->getCentroid().x * pole->getCentroid().x) +
                    (pole->getCentroid().y * pole->getCentroid().y);

            if (actualSDistance < minSDistance)
            {
                nearest = pole;
                minSDistance = actualSDistance;
            }
        }
    }

    //std::cout << "The nearest " << (onRight ? "right" : "left") << " pole is: " << nearest->ID() << std::endl;
}

void PoleExtractor::clusterize(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                               std::shared_ptr< std::vector<pcl::PointIndices> > &clusterIndices) const
{
    std::vector<pcl::PointIndices> vector;
    clusterIndices = std::make_shared< std::vector<pcl::PointIndices> >(vector);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr
            tree (new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud (source);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ>
            ec;

    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (min_cluster_size_);
    ec.setMaxClusterSize (max_cluster_size_);
    ec.setSearchMethod (tree);
    ec.setInputCloud (source);
    ec.extract (*clusterIndices);

    // TODO: Clean too wide clusters if it result necessary

}

void PoleExtractor::polesFromClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                      const std::shared_ptr<const std::vector<pcl::PointIndices> > &clusterIndices,
                                      std::shared_ptr< std::vector< std::shared_ptr<Pole> > > &polesVector) const
{
    polesVector.reset();
    std::vector< std::shared_ptr<Pole> > vector;
    polesVector = std::make_shared< std::vector< std::shared_ptr<Pole> > >(vector);

    std::vector<pcl::PointIndices>::const_iterator
            it = clusterIndices->begin();

    for (; it != clusterIndices->end(); ++it)
    {
        std::vector<cv::Point2f>
                pointsVector;

        pointsVector.reserve(it->indices.size());

        for (std::vector<int>::const_iterator pit = it->indices.begin ();
             pit != it->indices.end ();
             pit++)
        {
            cv::Point2f pt;

            pt.x = source->points[*pit].x;
            pt.y = source->points[*pit].y;

            pointsVector.push_back(pt);
        }
        /*

        // Create the new pole and add it to the vector
        // without the new at the } the pole will be destrojed and the ID repushed to the stack...
        // the data anyway is not lost
        std::shared_ptr<Pole> p( new Pole(pointsVector) );
        p->setStatus(Pole::JUST_SCANNED);
        polesVector->push_back(p);
        */
        // Avoid new like the hell!
        Pole p(std::make_shared< std::vector<cv::Point2f> >(pointsVector));
        std::shared_ptr<Pole>
                p_s = std::make_shared<Pole>(p);

        // Hack to avoid the calling of the destructor
        p_s->requestID();
        p_s->setStatus(Pole::JUST_SCANNED);

        polesVector->push_back(p_s);
    }
}

/*void PoleExtractor::compute_centroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                                      const std::vector<pcl::PointIndices> &clusterIndices,
                                      std::vector<cv::Point2f> &centroidsVector)
{
    // Reserve the space for centroids
    centroidsVector.clear();
    centroidsVector.reserve(clusterIndices.size());

    // Iterate clusters and compute centroids
    std::vector<pcl::PointIndices>::const_iterator idx_it = clusterIndices.begin();
    for (; idx_it != clusterIndices.end(); idx_it++)
    {
        // Compute the value once for all
        double value = 1 / idx_it->indices.size();

        cv::Point2f centroid(0,0);

        // Iterate all indices, the centroid is the arithmetic median of all points
        std::vector<int>::const_iterator pit = idx_it->indices.begin ();
        for (; pit != idx_it->indices.end (); pit++)
        {
            centroid.x = centroid.x + source->points[*pit].x * value;
            centroid.y = centroid.y + source->points[*pit].y * value;
        }

        // Save the point
        centroidsVector.push_back(centroid);
    }
}*/

} // namespace vineyard
