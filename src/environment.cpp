/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // DONE set renderScene to false to disable rendering the highway
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // DONE:: Create lidar sensor
    Lidar &lidar = *new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = lidar.scan();
    // renderRays(viewer, lidar.position, pointCloud); // experiment rendering rays
    // renderPointCloud(viewer, pointCloud, "pointCloud"); // experiment rendering input point cloud

    // DONE:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointCloudProcessor;

    // DONE Segment and render point clouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
              pcl::PointCloud<pcl::PointXYZ>::Ptr>
        segmentCloud = pointCloudProcessor.SegmentPlane(pointCloud, 150, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0)); // expirement rendering obstacle points
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); // expirement rendering ground points
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointCloudProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0),
                                 Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster : cloudClusters) {
      std::cout << "cluster size ";
      
      pointCloudProcessor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId%colors.size()]);

      Box box = pointCloudProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);

      ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> &pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::ConstPtr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // renderPointCloud(viewer, inputCloud, "inputCloud"); // expirement rendering input point cloud

    // Filter point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI.FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-20, -6, -4, 1), Eigen::Vector4f (30, 6.5, 5, 1));
    // renderPointCloud(viewer, filteredCloud, "inputCloud"); // expirement rendering filtered point cloud

    // Segment point clouds
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
              pcl::PointCloud<pcl::PointXYZI>::Ptr>
    segmentCloud = pointProcessorI.SegmentPlane(filteredCloud, 25, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0)); // expirement rendering obstacle points
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0)); // expirement rendering ground points

    // Cluster obstacles
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.5, 10, 500);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0),
                                 Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cluster : cloudClusters) {
      std::cout << "cluster size ";
      
      pointProcessorI.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                       colors[clusterId%colors.size()]);

      Box box = pointProcessorI.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);

      ++clusterId;
    }
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // simpleHighway(viewer); // experiment simple highway
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;

    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

    // Expiremenmt single frame
    // inputCloud = pointProcessorI.loadPcd(
    //     "../src/sensors/data/pcd/data_1/0000000000.pcd");
    // cityBlock(viewer, pointProcessorI, inputCloud);
    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 

    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloud = pointProcessorI.loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloud);

      streamIterator++;
      if (streamIterator == stream.end()) 
        streamIterator = stream.begin();

      viewer->spinOnce();
    } 
}