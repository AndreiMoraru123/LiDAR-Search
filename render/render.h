#ifndef RENDER_H
#define RENDER_H
#include <pcl/visualization/pcl_visualizer.h>
#include "box.h"
#include <iostream>
#include <utility>
#include <vector>
#include <string>

struct Color  // Color is a struct that contains RGB info
{
    float r, g, b;
    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};

enum CameraAngle
{
    XY, TopDown, Side, FPS
};


void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::string& name, Color color = Color(1,1,1));
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id, Color color = Color(1,0,0), float opacity=1);
void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, const BoxQ& box, int id, Color color = Color(1,0,0), float opacity=1);

#endif /* RENDER_H */