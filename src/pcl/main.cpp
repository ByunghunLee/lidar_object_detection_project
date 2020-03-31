#include <iostream>
#include "render.h"
#include "pointCloudProcessor.h"
#include "pointCloudProcessor.cpp"


template <typename PointT>
void processAndRenderPointCloud
(
        Renderer& renderer,
        const PointCloudProcessor<PointT>& point_processor,
        const typename pcl::PointCloud<PointT>::Ptr& input_cloud)
{
    auto filtered_cloud = point_processor.FilterCloud(input_cloud, 0.5f, Eigen::Vector4f(-10, -6, -3, 1), Eigen::Vector4f(20, 6, 3, 1));
    auto segmented_cloud = point_processor.SegmentPlane(filtered_cloud, 100, 0.2f);

    //point_processor.SeparateClouds(filtered_cloud, 100, 0.2);
    renderer.renderPointCloud(segmented_cloud.second, "segmented_cloud", Color(0,1,0));

    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters =
            point_processor.Clustering(segmented_cloud.first, 0.7, 10, 1000);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1), Color(1,0,1), Color(0,1,1)};

    for(const auto& cluster : cloudClusters)
    {
        renderer.renderPointCloud(cluster,"obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        Box box = point_processor.BoundingBox(cluster);
        renderer.renderBox(box, clusterId);
        ++clusterId;
    }


}

int main()
{
    CameraAngle setAngle = XY;
    Renderer renderer("3D Viewer");
    renderer.initCamera(setAngle);

    auto point_processor = PointCloudProcessor<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = point_processor.streamPcd("/Users/byunghun.ee/ClionProjects/lidar_object_detecton_project/data/pcd/data_2");

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

    while (!renderer.wasViewerStopped())
    {
        renderer.clearViewer();

        input_cloud = point_processor.loadPcd(streamIterator->string());
        processAndRenderPointCloud(renderer, point_processor, input_cloud);
        //renderer.renderPointCloud(input_cloud, "planeCloud", Color(0,1,0));
        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        renderer.spinViewerOnce();
    }

    return 0;
}
