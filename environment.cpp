#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obstacles = true;
    bool render_plane = true;
    bool render_box = false;
    bool render_quaternion_box = true;
    bool render_rays = false;
    bool render_cluster = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);

    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    if (render_rays) {
        renderRays(viewer, lidar->position, inputCloud);
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }

    auto* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(inputCloud, 100, 0.2);
    if (render_obstacles) {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    }
    if (render_plane) {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    for(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor->numPoints(cluster);

        if (render_cluster) {
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
        }

        if (render_box) {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if (render_quaternion_box) {
            BoxQ box = pointProcessor->BoundingBoxQ(cluster);
            renderBox(viewer, box, clusterId);
        }

        ++clusterId;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters();

    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    bool render_obstacles = false;
    bool render_plane = true;
    bool render_box = true;
    bool render_quaternion_box = false;
    bool render_cluster = true;

    if (renderScene) {
        renderPointCloud(viewer, inputCloud, "inputCloud");
    }
    // Filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));

    // Segmentation using custom method vs PCL
//     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane3D(filterCloud, 100, 0.2);

    if (render_obstacles) {
        renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    }
    if (render_plane) {
        renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    }

    // Clustering using custom method vs PCL
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.4, 10, 500);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering3D(segmentCloud.first, 0.4, 10, 500);

    int clusterId = 0;

    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloudClusters)
    {

        if (render_cluster) {
            renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        if (render_box) {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if (render_quaternion_box) {
            BoxQ box = pointProcessorI->BoundingBoxQ(cluster);
            renderBox(viewer, box, clusterId);
        }

        ++clusterId;
    }
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->getRenderWindow()->GlobalWarningDisplayOff();
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    auto* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd(R"(D:\\Lidar\\sensors\\data\\pcd\\data_1\\)");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
//        renderPointCloud(viewer, inputCloudI, "inputCloud"); // to stream raw data
        CityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

}