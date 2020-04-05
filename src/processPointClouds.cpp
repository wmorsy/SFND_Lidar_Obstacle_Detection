// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const pcl::PointIndices::Ptr inliers, const typename pcl::PointCloud<PointT>::Ptr cloud) const
{
    // DONE: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>(*cloud, inliers->indices));
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    // Create the filtering object
    typename pcl::ExtractIndices<PointT> extract;

    // Extract the outliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::RansacPlane(pcl::PointIndices &inliersResult, const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    //DONE implement Ransac 3D
	srand(time(NULL));
    auto &inliersResultIndices = inliersResult.indices;

	// For max iterations 
    while (maxIterations--) {
        // Randomly sample subset and fit line
        std::unordered_set<int> inliers;

        while (inliers.size() < 3)
          inliers.insert(rand() % (cloud->points.size()));

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); //i
        float b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1); //j
        float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); //k
        float d = -1*(a*x1 + b*y1 + c*z1);

        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for (int index = 0; index < cloud->points.size(); index++) {
          if (inliers.count(index) > 0) 
            continue;

          PointT point = cloud->points[index];
          float x4 = point.x;
          float y4 = point.y;
          float z4 = point.z;
          float d = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

          if (d <= distanceTol)
            inliers.insert(index);
        }

        // Return indicies of inliers from fitted line with most inliers
        if(inliers.size()>inliersResultIndices.size()) {
            inliersResultIndices.clear();
            inliersResultIndices.insert(inliersResultIndices.end(), inliers.begin(), inliers.end());
        }
    }
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(const typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold, bool useRansac)
{
    // DONE:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::string segType;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use RANSAC segmentation
    if(useRansac) {
        segType = "RANSAC";
        RansacPlane(*inliers, cloud, maxIterations, distanceThreshold);
    }
    // Use PCL Segmentation
    else {
        segType = "PCL";
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);

        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }
    }
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << segType << " plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}