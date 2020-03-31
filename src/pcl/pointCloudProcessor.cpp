#include "pointCloudProcessor.h"
//constructor:
template<typename PointT>
PointCloudProcessor<PointT>::PointCloudProcessor() {}

//de-constructor:
template<typename PointT>
PointCloudProcessor<PointT>::~PointCloudProcessor() {}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudProcessor<PointT>::FilterCloud
(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float filterRes, Eigen::Vector4f minPoint,
        Eigen::Vector4f maxPoint) const
{
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::SegmentPlane
( const typename pcl::PointCloud<PointT>::Ptr cloud, size_t max_iterations, double_t distance_threshold) const
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    auto inliers_set = RansacPlane(cloud, max_iterations, distance_threshold);
    inliers->indices.insert(inliers->indices.begin(), inliers_set.begin(), inliers_set.end());



    typename pcl::PointCloud<PointT>::Ptr road_cloud{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr obstacles_cloud{new pcl::PointCloud<PointT>()};


    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers (road points)
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road_cloud);

    // Extract the obstacle points
    extract.setNegative(true);
    extract.filter(*obstacles_cloud);

    return std::make_pair(obstacles_cloud, road_cloud);
}

template <typename PointT>
std::unordered_set<int> PointCloudProcessor<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol) const
{
    std::unordered_set<int> inliers_result;
    srand(time(NULL));

    std::vector<float> v3;
    v3.reserve(3);

    while(maxIterations--)
    {
        std::unordered_set<int> inliers;

        while(inliers.size() < 3)
        {
            inliers.insert(rand()%(cloud->points.size()));
        }

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

        std::vector<float> v1 = {x2-x1, y2-y1, z2-z1};
        std::vector<float> v2 = {x3-x1, y3-y1, z3-z1};

        v3[0] = v1[1] * v2[2] - v1[2] * v2[1];
        v3[1] = v1[2] * v2[0] - v1[0] * v2[2];
        v3[2] = v1[0] * v2[1] - v1[1] * v2[0];

        for(int index = 0; index < cloud->points.size(); index++)
        {
            if(inliers.count(index) > 0 ) continue;

            pcl::PointXYZI point = cloud->points[index];

            float x = point.x;
            float y = point.y;
            float z = point.z;

            float d = fabs(v3[0]*x + v3[1]*y + v3[2]*z -(v3[0]*x1 + v3[1]*y1 + v3[2]*z1)) /sqrt(v3[0]*v3[0] + v3[1]*v3[1] + v3[2]*v3[2]);

            if(d <= distanceTol) inliers.insert(index);

            if(inliers.size()  > inliers_result.size())
            {
                inliers_result = inliers;
            }
        }
    }

    return inliers_result;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> PointCloudProcessor<PointT>::Clustering
(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance,
        int minSize,
        int maxSize) const
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    for(pcl::PointIndices getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index: getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    return clusters;
}
//
//
template<typename PointT>
Box PointCloudProcessor<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) const
{

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
typename pcl::PointCloud<PointT>::Ptr PointCloudProcessor<PointT>::loadPcd(std::string file)
{
    std::cout << file << std::endl;
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from " + file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> PointCloudProcessor<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    sort(paths.begin(), paths.end());

    return paths;
}