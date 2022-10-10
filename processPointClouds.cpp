#include <filesystem>
#include "processPointClouds.h"
#include <Eigen/Dense>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() = default;


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() = default;


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    // remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point: indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{

    typename pcl::PointCloud<PointT>::Ptr obstCloud{new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>()};

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}


template<typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const pcl::PointIndices& getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box{};
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

// PCA bounding box
template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    BoxQ box;
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());

    pcl::PointCloud<PointT> cloudPointsProjected;
    pcl::transformPointCloud(*cluster, cloudPointsProjected, projectionTransform);
    pcl::getMinMax3D(cloudPointsProjected, minPoint, maxPoint);

    box.bboxQuaternion = eigenVectorsPCA;
    box.bboxTransform = pcaCentroid.head<3>();
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients ()};
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;

}

//template<typename PointT>
//std::vector<std::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
//{
//
//    std::vector<std::filesystem::path> paths;
//
//    // read from dataPath
//    for (const auto & entry : std::filesystem::directory_iterator(dataPath))
//        paths.push_back(entry.path());
//
//    // sort files in accending order so playback is chronological
//    sort(paths.begin(), paths.end());
//
//    return paths;
//
//}

//template<typename PointT>
//std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(fs::path dataPath)
//{
//
//    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
//
//    // sort files in accending order so playback is chronological
//    sort(paths.begin(), paths.end());
//
//    return paths;
//
//}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
        std::string dataPath) {
    std::vector<boost::filesystem::path> paths(
            boost::filesystem::directory_iterator{dataPath},
            boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations--) {
        // Randomly sample subset and fit line
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

        float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float d = -(a * x1 + b * y1 + c * z1);
        const float norm = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0)
                continue;

            PointT point = cloud->points[index];
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / norm;

            if (dist <= distanceThreshold)
                inliers.insert(index);
        }

        if (inliers.size() > inliersResult.size()) inliersResult = inliers;
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}


static void recursiveClustering(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, TreeKD* tree, float distanceTol)
{
    processed[indice] = true;
    cluster.push_back(indice);
    std::vector<int> nearest = tree->search(points[indice], distanceTol);

    for (int id : nearest)
    {
        if (!processed[id])
            recursiveClustering(id, points, cluster, processed, tree, distanceTol);
    }
}

template <typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering3D(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    std::vector<std::vector<int>> clusters;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;
    std::vector<bool> processed(cloud->points.size(), false);
    std::vector<std::vector<float>> distances;

    TreeKD *tree = new TreeKD;

    for (int i = 0; i < cloud->points.size(); i++) {
        tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
        distances.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }


    for (int i = 0; i < distances.size(); i++) {
        if (processed[i])
            continue;

        std::vector<int> cluster;

        recursiveClustering(i, distances,  cluster, processed, tree, clusterTolerance);
        clusters.push_back(cluster);
    }

    delete tree;

    for (const std::vector<int> &cluster: clusters) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int indice: cluster)
            cloudCluster->points.push_back(cloud->points[indice]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        if (cloudCluster->points.size() >= minSize && cloudCluster->points.size() <= maxSize)
            cloudClusters.push_back(cloudCluster);
    }


    return cloudClusters;

}