#ifndef MYCLUSTER_H_
#define MYCLUSTER_H_

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <string>
#include "mykdtree.h"

void myProximity(std::vector<std::vector<float>> points,int i,std::vector<int>& cluster,std::vector<int>& mark,KdTree* tree, float distanceTol);
std::vector<std::vector<int>> myEuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

#endif /* MYCLUSTER_H_ */