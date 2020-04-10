// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "quiz/cluster/kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::ConstPtr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::ConstPtr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(const pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::ConstPtr cloud) const;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::ConstPtr cloud, int maxIterations, float distanceThreshold, bool usePclSegmentation=false);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize, const bool usePclClustering=false);

    Box BoundingBox(typename pcl::PointCloud<PointT>::ConstPtr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    void RansacPlane(pcl::PointIndices &inliersResult,  typename pcl::PointCloud<PointT>::ConstPtr cloud, int maxIterations, float distanceTol);

    void clusterHelper(const int index, const std::vector<std::vector<float>> &points, pcl::PointIndices &cluster, std::vector<bool> &processed, const KdTree *tree, const float distanceTol, int minSize, int maxSize);

    std::vector<pcl::PointIndices> euclideanCluster(const std::vector<std::vector<float>> &points, const KdTree *tree, const float distanceTol, int minSize, int maxSize);
};
#endif /* PROCESSPOINTCLOUDS_H_ */