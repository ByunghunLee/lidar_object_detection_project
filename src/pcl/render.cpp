/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"

Renderer::Renderer(const std::string &name) :  _viewer{new pcl::visualization::PCLVisualizer()}, _count_rays{0}
{
    _viewer->setWindowName(name);
}

void Renderer::initCamera(CameraAngle setAngle)
{

    _viewer->setBackgroundColor (0, 0, 0);
    // set camera position and angle
    _viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : _viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : _viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : _viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : _viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        _viewer->addCoordinateSystem (1.0);
}

void Renderer::renderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color)
{
    _viewer->addPointCloud<pcl::PointXYZ> (cloud, name);
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}

void Renderer::renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{
    if(color.r==-1)
    {
        // Select color based off of cloud intensity
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
        _viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
        // Select color based off input value
        _viewer->addPointCloud<pcl::PointXYZI> (cloud, name);
        _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }

    _viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
}
void Renderer::renderBox(Box box, int id, Color color, float opacity)
{
    if(opacity > 1.0)
        opacity = 1.0;
    if(opacity < 0.0)
        opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    _viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    //viewer->addCube(box.bboxTransform, box.bboxQuaternion, box.cube_length, box.cube_width, box.cube_height, cubeFill);
    _viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cubeFill);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    _viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity*0.3, cubeFill);
}
void Renderer::clearViewer()
{
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
}
bool Renderer::wasViewerStopped() const
{
    return _viewer->wasStopped();
}
void Renderer::spinViewerOnce() const
{
    return _viewer->spinOnce();
}