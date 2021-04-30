#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

/*
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
        // car1.render(viewer);
        // car2.render(viewer);
        // car3.render(viewer);
    }

    return cars;
} 
*/

//L4
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // L4-C2:: Load pcd data
    // ProcessPointClouds<pcl::PointXYZI> pointProcessorI; // Stack: deleted after function returns A.f() fast but not free
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI.loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // L4-C5:: Filtering (voxel)
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI.FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30, 7, 2, 1));
    // renderPointCloud(viewer, filterCloud, "filterCloud");

    // L4-C6:: Add seg & cluster
    // 1. Segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.SegmentPlane(filterCloud, 100, 0.2);
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud");
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // 2. Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.53, 10, 500);
    int clusterId = 0; 
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%3]);

        // L3-C9:: Render he boxes
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer, box, clusterId, Color(0.9, 0.175, 0));
        // EOTD L3-C9
        clusterId++;
    }


}
/*
void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) // My Vis Function
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    // bool renderScene = true;
    bool renderScene = false;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // L1-C15:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    // L1-C16:: Using the lidar object && Render laser rays
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud = lidar->scan();
    // renderRays(viewer, lidar->position, scanCloud); 

    // // L1-C19:: Render point cloud instead of rays
    // renderPointCloud(viewer, scanCloud, "L1-C19");

    // L2-C3:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // Stack: deleted after function returns

    // L2-C5:: Render segmented clouds
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(scanCloud, 100, 0.2); // Distance threshold: ground vertical
    // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // L3-C3:: Color code clusters
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // L3-C9:: Render he boxes
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        // EOTD L3-C9
        ++clusterId;
    }
    // EOTD L3-C3
}
*/

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
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);

    // L4-C2
    // cityBlock(viewer);

    // L4-C7:: Stream PCD
    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    auto streamItr = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI.loadPcd((*streamItr).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamItr++;
        if(streamItr == stream.end())
        {
            streamItr = stream.begin();
        }
        viewer->spinOnce ();
    } 
}