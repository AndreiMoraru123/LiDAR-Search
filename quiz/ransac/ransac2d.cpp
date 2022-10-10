//
// Created by Andrei on 07-Oct-22.
//

#include "../../render/render.h"
#include "../../processPointClouds.h"
#include "../../processPointClouds.cpp"


pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    // Add inliers
    float scatter = 0.6;
    for (int i = -5; i < 5; i++)
    {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = i + scatter * rx;
        point.y = i + scatter * ry;
        point.z = 0;

        cloud->points.push_back(point);
    }

    // Add outliers
    int numOutliers = 30;
    while (numOutliers--)
    {
        double rx = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        double ry = 2 * (((double) rand() / (RAND_MAX)) - 0.5);
        pcl::PointXYZ point;
        point.x = 5 * rx;
        point.y = 5 * ry;
        point.z = 0;

        cloud->points.push_back(point);

    }

    cloud->width = cloud->points.size();
    cloud->height = 1;

    return cloud;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i=0; i<=maxIterations; i++)
    {
        // random sample a subset and fit a line
        std::unordered_set<int> inliers;
        while (inliers.size() < 2)
            inliers.insert(rand() % (cloud->points.size()));

        float x1, y1, x2, y2;
        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        itr++;

        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        // distance between every point and fitted line
        float a = (y1 - y2);
        float b = (x2 - x1);
        float c = (x1*y2 - x2*y1);
        float d = sqrt(a*a + b*b);

        for (int index=0; index<cloud->points.size(); index++)
        {
            if (inliers.count(index) > 0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x3 = point.x;
            float y3 = point.y;

            float dist = fabs(a*x3 + b*y3 + c) / d;

            if (dist <= distanceTol)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size())  inliersResult = inliers;
    }

    return inliersResult;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    return pointProcessor.loadPcd(R"(D:\\Lidar\\sensors\\data\\pcd\\simpleHighway.pcd)");

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i = 0; i <= maxIterations; i++) {
        // random sample a subset and fit a line
        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % (cloud->points.size()));

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

        // distance between every point and fitted line
        float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float d = -(a * x1 + b * y1 + c * z1);
        const float norm = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0)
                continue;

            pcl::PointXYZ point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / norm;

            if (dist <= distanceTol)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size()) inliersResult = inliers;
    }

    return inliersResult;
}

int main()
{
    // Create viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, 15, 0, 1, 0);

    // Create data
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

    // visualize data
    // renderPointCloud(viewer, cloud, "data");

    std::unordered_set<int> inliers = Ransac3D(cloud, 200, 0.25);

    // visualize inliers and outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

    for (int index = 0; index < cloud->points.size(); index++) {
        pcl::PointXYZ point = cloud->points[index];
        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }
}
