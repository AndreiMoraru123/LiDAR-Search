#include "Tree_I_KD.h"
#include "Tree_I_KD.cpp"
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "../processPointClouds.h"
#include "../processPointClouds.cpp"
#include <pcl/visualization/pcl_visualizer.h>
#include "../render/render.cpp"


using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;
template class KD_TREE<pcl::PointXYZ>;


void initializeCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();

    int distance = 20;

    switch (setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }

    if (setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void generate_box(BoxPointType &boxpoint, const PointType &center_pt, vector<float> box_lengths) {
    float &x_dist = box_lengths[0];
    float &y_dist = box_lengths[1];
    float &z_dist = box_lengths[2];

    boxpoint.vertex_min[0] = center_pt.x - x_dist;
    boxpoint.vertex_max[0] = center_pt.x + x_dist;
    boxpoint.vertex_min[1] = center_pt.y - y_dist;
    boxpoint.vertex_max[1] = center_pt.y + y_dist;
    boxpoint.vertex_min[2] = center_pt.z - z_dist;
    boxpoint.vertex_max[2] = center_pt.z + z_dist;
}

void colorize( const PointVector &pc, pcl::PointCloud<pcl::PointXYZRGB> &pc_colored, const std::vector<int> &color) {
    int N = pc.size();

    pc_colored.clear();
    pcl::PointXYZRGB pt_tmp;

    for (int i = 0; i < N; ++i) {
        const auto &pt = pc[i];
        pt_tmp.x = pt.x;
        pt_tmp.y = pt.y;
        pt_tmp.z = pt.z;
        pt_tmp.r = color[0];
        pt_tmp.g = color[1];
        pt_tmp.b = color[2];
        pc_colored.points.emplace_back(pt_tmp);
    }
}



float test_dist(PointType a, PointType b)
{
    float dist = 0.0f;
    dist = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z);
    return dist;
}


void RadiusVision( pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<pcl::PointXYZ>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{

    // filter cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));

    // segment cloud
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorI->SegmentPlane3D(filterCloud, 100, 0.2);

    // cluster cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->Clustering3D(segmentCloud.first, 0.4, 10, 500);

    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType> (0.3, 0.6, 0.2));
    KD_TREE<PointType> &kdtree = *kdtree_ptr;

    auto start = std::chrono::high_resolution_clock::now();
    kdtree.Build((*inputCloud).points);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Tree building time: " << duration.count() << "ms" << std::endl;
    std::cout << "Number of points: " << kdtree.validnum() << std::endl;

    PointType ball_center_pt;
    ball_center_pt.x = 0;
    ball_center_pt.y = 0;
    ball_center_pt.z = 0;

    float radius = 10;

    start = std::chrono::high_resolution_clock::now();
    PointVector SearchedPointsRadius;
    kdtree.Radius_Search(ball_center_pt, radius, SearchedPointsRadius);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Radius search time: " << duration.count() << "ms" << " with points " << SearchedPointsRadius.size() << std::endl;

    std::vector<int> color = {255, 255, 255};

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_radius_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

//    pcl::visualization::PointCloudColorHandlerGenericField<PointType> color_handler(inputCloud, "x");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(inputCloud, 0, 255, 0);
    colorize(SearchedPointsRadius, *searched_radius_colored, color);


    viewer->addPointCloud<PointType>(inputCloud, color_handler, "src");
//    viewer->addPointCloud<PointType>(segmentCloud.second, color_handler, "src");
    viewer->addPointCloud<pcl::PointXYZRGB> (searched_radius_colored, "cloud_colored_radius");

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(1,0.84,0), Color(1,1,0.5)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (test_dist(cluster->points[0], ball_center_pt) < radius * radius)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
        }
    }

    viewer->addText("Yellow: Obstacles & Pedestrians", 10, 10, 16, 1.0, 1.0, 1.0, "cyan");
    viewer->addText("White: Detection Radius", 10, 30, 16, 1.0, 1.0, 1.0, "purple");
    viewer->addText("Red: Bounding Boxes", 10, 50, 16, 1.0, 1.0, 1.0, "orange");
}

// same function as radiusvision but with box search call BoxVision

void BoxVision( pcl::visualization::PCLVisualizer::Ptr viewer, ProcessPointClouds<pcl::PointXYZ>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {

    // filter cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f,
                                                                                   Eigen::Vector4f(-10, -5, -2, 1),
                                                                                   Eigen::Vector4f(30, 8, 1, 1));

    // segment cloud
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessorI->SegmentPlane3D(
            filterCloud, 100, 0.2);

    // cluster cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->Clustering3D(segmentCloud.first,
                                                                                                   0.4, 10, 500);

    KD_TREE<PointType>::Ptr kdtree_ptr(new KD_TREE<PointType>(0.3, 0.6, 0.2));
    KD_TREE<PointType> &kdtree = *kdtree_ptr;

    auto start = std::chrono::high_resolution_clock::now();
    kdtree.Build((*inputCloud).points);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Tree building time: " << duration.count() << "ms" << std::endl;
    std::cout << "Number of points: " << kdtree.validnum() << std::endl;

    PointType center_pt;
    center_pt.x = 0.0;
    center_pt.y = 0.0;
    center_pt.z = 0.0;
    BoxPointType boxpoint;
    std::vector<float> box_lengths = {10.00, 10.00, 10.00};
    generate_box(boxpoint, center_pt, box_lengths);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

    start = std::chrono::high_resolution_clock::now();
    PointVector SearchedPointsBox;
    kdtree.Box_Search(boxpoint, SearchedPointsBox);
    stop = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Box search time: " << duration.count() << "ms" << " with points " << SearchedPointsBox.size() << std::endl;

    std::vector<int> color = {0, 0, 255};

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_colored(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr searched_box_colored(new pcl::PointCloud<pcl::PointXYZRGB>);

//    pcl::visualization::PointCloudColorHandlerGenericField<PointType> color_handler(inputCloud, "x");
    pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(inputCloud, 0, 255, 0);
    colorize(SearchedPointsBox, *searched_box_colored, color);


    viewer->addPointCloud<PointType>(inputCloud, color_handler, "src");
//    viewer->addPointCloud<PointType>(segmentCloud.second, color_handler, "src");
    viewer->addPointCloud<pcl::PointXYZRGB> (searched_box_colored, "cloud_colored_radius");

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,1,0), Color(1,0.84,0), Color(1,1,0.5)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (test_dist(cluster->points[0], center_pt) < box_lengths[0] * box_lengths[1])
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId % colors.size()]);
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
            ++clusterId;
        }
    }

    viewer->addText("Yellow: Obstacles & Pedestrians", 10, 10, 16, 1.0, 1.0, 1.0, "cyan");
    viewer->addText("White: Detection Radius", 10, 30, 16, 1.0, 1.0, 1.0, "purple");
    viewer->addText("Red: Bounding Boxes", 10, 50, 16, 1.0, 1.0, 1.0, "orange");

}

int main (int argc, char** argv)
{
    std::cout<<"starting enviroment"<<std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->getRenderWindow()->GlobalWarningDisplayOff();
    CameraAngle setAngle = XY;
    initializeCamera(setAngle, viewer);

    auto* pointProcessorI = new ProcessPointClouds<pcl::PointXYZ>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(R"(D:\\Lidar\\sensors\\data\\pcd\\data_2\\)");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloudI;

    viewer->setSize(900, 500);
    while(!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
//        renderPointCloud(viewer,inputCloudI,"inputCloudI");
//        RadiusVision(viewer, pointProcessorI, inputCloudI);
        BoxVision(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

}