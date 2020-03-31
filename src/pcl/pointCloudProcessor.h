// PCL lib Functions for processing point clouds 
#pragma once

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
#include <unordered_set>
#include "box.h"

template<typename PointT>
class PointCloudProcessor
{

public:
    PointCloudProcessor();
    ~PointCloudProcessor();

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);
    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) const;
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>SegmentPlane(const typename  pcl::PointCloud<PointT>::Ptr cloud, size_t max_iterations, double_t distance_threshold) const;
    std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) const;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) const;
    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) const;
};
#endif /* PROCESSPOINTCLOUDS_H_ */