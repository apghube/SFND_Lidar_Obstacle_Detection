/* \author Amruta Ghube */

#include "cluster.h"

void myProximity(std::vector<std::vector<float>> points,int i,std::vector<int>& cluster,std::vector<int>& mark,KdTree* tree, float distanceTol)
{
   mark[i] =1; //mark point to be processed
   cluster.push_back(i); //target point for search in K-D Tree
   std::vector<int> nearbypoints = tree->search(points[i],distanceTol);
  	for(std::vector<int>::iterator it=nearbypoints.begin(); it != nearbypoints.end(); ++it)
	  {
		  if(mark[*it] == 0) //if point has not been processed
		  {
			  myProximity(points,*it,cluster,mark,tree,distanceTol);
		  }
		  
	  }
}

std::vector<std::vector<int>> myEuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	std::vector<std::vector<int>> clusters; //list of clusters
	std::vector<int> mark;
	mark.assign(points.size(),0); //mark each point to be un-processed
	for(int i=0; i< points.size(); i++)
	{
        if(mark[i] == 0) //if point has not been processed
		{
			std::vector<int> cluster;
			myProximity(points,i,cluster,mark,tree,distanceTol);
			clusters.push_back(cluster);

		}
	}
	return clusters;
} 	

