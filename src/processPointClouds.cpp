// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"



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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
   pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>); //planar cloud = road
   pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>); //obstacle cloud

  int nr_points = (int) cloud->points.size ();
  // While 30% of the original cloud is still there
  while (cloud->points.size () > 0.3 * nr_points)
  {
    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false); //normal filter behavior
    extract.filter (*plane_cloud);

    // Create the filtering object
    extract.setNegative (true); //inverted filter behavior
    extract.filter (*obstacle_cloud);
    cloud.swap (obstacle_cloud);
  }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}
/*RansacPlane Integration in ProcessPointClouds*/
template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	// For max iterations 
    for(int mi=0;mi<maxIterations;mi++)
	{

	  // Randomly sample subset and fit line
	  std::unordered_set<int> randpoint;
	  int n=3; //three points to form a plane
	  
	  while(n>0)
	  {
        randpoint.insert(rand () % cloud->points.size());
		n--;
	  }
      
	  auto it=randpoint.begin();
	  float x1=cloud->points[*it].x; 
	  float y1=cloud->points[*it].y;
      float z1=cloud->points[*it].z;
	  it++;
	  float x2=cloud->points[*it].x;
	  float y2=cloud->points[*it].y;
      float z2=cloud->points[*it].z;
      it++;
      float x3=cloud->points[*it].x;
	  float y3=cloud->points[*it].y;
      float z3=cloud->points[*it].z;
      
      float i=( ((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1)) );
      float j=( ((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1)) );
      float k=( ((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1)) );

	  float A= i;
	  float B= j;
	  float C= k;
      float D=-( (i*x1) + (j*y1) + (k*z1) );

	  for(int idx=0; idx< cloud->points.size();idx++)
	  {

        if(randpoint.count(idx)>0) //ignore if point has used to form plane
		   continue;

		pcl::PointXYZ point4;
        point4=cloud->points[idx];
		float x4=point4.x;
		float y4=point4.y;
        float z4=point4.z;
		
        // Measure distance between every point and fitted line
	    float distance ;
	    distance = fabs(A*x4 + B*y4 + C*z4 + D)/sqrt(A*A + B*B + C*C);
        // If distance is smaller than threshold count it as inlier
	    if(distanceTol >= distance)
	    {
          randpoint.insert(idx); //insert the indices of the inliers
	    }
	  }
	  if(randpoint.size()>inliersResult.size())
	    inliersResult=randpoint;
	}

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;

}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    

    /*// TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE); //model type is Plane
    seg.setMethodType (pcl::SAC_RANSAC);   //Method used for segmentation is RANSAC
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    
    // Segment the largest planar component from the cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);*/

    std::unordered_set<int> inliersSet = RansacPlane(cloud, maxIterations, distanceThreshold);
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int i : inliersSet)
    {
      inliers->indices.push_back(i);
    }
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given point cloud." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

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