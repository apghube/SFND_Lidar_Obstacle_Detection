/* \author Amruta Ghube */

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include "kdtree.h"

void myProximity(std::vector<std::vector<float>> points,int i,std::vector<int>& cluster,std::vector<int>& mark,KdTree* tree, float distanceTol);
std::vector<std::vector<int>> myEuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif /* CLUSTER_H_ */