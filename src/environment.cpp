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
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar_obj = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input = lidar_obj->scan();
   // renderRays(viewer,lidar_obj->position,cloud_input);
  // renderPointCloud(viewer,cloud_input,"hdl64", Color(0,0,1));

    // TODO:: Create point processor
   //ProcessPointClouds<pcl::PointXYZ> pointProc1;  //object is created on stack
    ProcessPointClouds<pcl::PointXYZ>* pointProc = new ProcessPointClouds<pcl::PointXYZ>(); //object is created on heap
    
    /* Cloud Segmentation and Obstacle Filtering */
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProc->SegmentPlane(cloud_input, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0)); //r,g,b
    //renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    /* Cloud Clustering */

   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProc->Clustering(segmentCloud.first, 1, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
       {
            std::cout << "cluster size ";
            pointProc->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
            Box box = pointProc->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
       }
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderFilterCloud = false;
    bool renderSegmentCloud = false;
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcI = new ProcessPointClouds<pcl::PointXYZI>(); //object created on heap
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  if(renderScene)
    renderPointCloud(viewer,inputCloud,"inputCloud");

  // ---------------------------------------------------------------------
  // -----Downsampling the Cityblock PCD using Voxel grid filter     -----
  // ---------------------------------------------------------------------
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-8, -4, -2, 1), Eigen::Vector4f (30,6,1, 1));
  if(renderFilterCloud)
    renderPointCloud(viewer,filterCloud,"filterCloud");

  // ---------------------------------------------------------------------
  // -----Segmentation: road and obstacles -------------------------------
  //-----------------------------------------------------------------------

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcI->SegmentPlane(filterCloud, 25, 0.2); 
  if(renderSegmentCloud)
    {
        renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0)); //r,g,b
        renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    }

  // ---------------------------------------------------------------------
  // ---------------------Cloud Clustering -------------------------------
  //-----------------------------------------------------------------------

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcI->Clustering(segmentCloud.first, 0.38, 9, 500); 
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
       {
            std::cout << "cluster size ";
            pointProcI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
            Box box = pointProcI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
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

    ProcessPointClouds<pcl::PointXYZI>* pointProcI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
   
    //simpleHighway(viewer);
   // cityBlock(viewer,pointProcI,inputCloud);
   
    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    }
}