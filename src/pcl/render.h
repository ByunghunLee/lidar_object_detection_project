#pragma once

#ifndef RENDER_H
#define RENDER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include "types.h"
#include <iostream>
#include <vector>
#include <string>

class Renderer
{
public:
    Renderer(const std::string &name);

    void initCamera(CameraAngle setAngle);
    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::string name, Color color = Color(1,1,1));
    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1));
    void renderBox(Box box, int id, Color color = Color(1,0,0), float opacity=1);

    void clearViewer();
    bool wasViewerStopped() const;
    void spinViewerOnce() const;

private:
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    int _count_rays;
};


#endif
