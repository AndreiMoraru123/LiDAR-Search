#include <filesystem>
#include "processPointClouds.h"
#include <Eigen/Dense>

// constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() = default;

// destructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() = default;

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    typename pcl::PointCloud<PointT>::Ptr voxel_cloud (new pcl::PointCloud<PointT>); // voxel cloud
    typename pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>); // cropped cloud

    pcl::VoxelGrid<PointT> voxel_grid; // voxel grid
    voxel_grid.setInputCloud(cloud); // set input cloud
    voxel_grid.setLeafSize(filterRes, filterRes, filterRes); // set voxel size
    voxel_grid.filter(*voxel_cloud); // filter cloud

    pcl::CropBox<PointT> crop_box(true); // crop box
    crop_box.setMin(minPoint); // set min point
    crop_box.setMax(maxPoint); // set max point
    crop_box.setInputCloud(voxel_cloud); // set input cloud
    crop_box.filter(*cropped_cloud); // filter cloud

    crop_box.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1)); // set min point
    crop_box.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1)); // set max point
    crop_box.setInputCloud(cropped_cloud); // set input cloud
    crop_box.setNegative(true); // set negative to true because we want to remove the roof
    crop_box.filter(*cropped_cloud); // filter cloud

    return cropped_cloud;
}

template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{

    typename pcl::PointCloud<PointT>::Ptr obstCloud{new pcl::PointCloud<PointT>()}; //obstacle cloud
    typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>()}; //plane cloud

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]); // add the point to the plane cloud

    pcl::ExtractIndices<PointT> extract; // Create the filtering object
    extract.setInputCloud(cloud); // Set the input point cloud
    extract.setIndices(inliers); // Set the indices to be extracted
    extract.setNegative(true); // Set to true to extract the indices
    extract.filter(*obstCloud); // Extract the inliers

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}

template<typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();  // start timer

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;  // vector of clusters

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);  // set input cloud

    std::vector<pcl::PointIndices> cluster_indices;  // vector of point indices
    pcl::EuclideanClusterExtraction<PointT> ec;  // euclidean cluster extraction
    ec.setClusterTolerance(clusterTolerance); // set cluster tolerance
    ec.setMinClusterSize(minSize);  // set min cluster size
    ec.setMaxClusterSize(maxSize);  // set max cluster size
    ec.setSearchMethod(tree);  // set search method
    ec.setInputCloud(cloud);  // set input cloud
    ec.extract(cluster_indices);  // extract clusters

    for (const pcl::PointIndices& getIndices: cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        // iterate through the points in the cluster
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]); // add the point to the cluster

        cloudCluster->width = cloudCluster->points.size();  // set width
        cloudCluster->height = 1;  // set height
        cloudCluster->is_dense = true;  // set density
        clusters.push_back(cloudCluster);  // add the cluster to the vector of clusters
    }

    auto endTime = std::chrono::steady_clock::now();  // end timer
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);  // get min and max points

    Box box{};  // create box
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
    BoxQ box; // create box
    PointT minPoint, maxPoint; // create min and max points
    pcl::getMinMax3D(*cluster, minPoint, maxPoint); // get min and max points

    Eigen::Vector4f pcaCentroid; // create pca centroid
    pcl::compute3DCentroid(*cluster, pcaCentroid); // compute pca centroid
    Eigen::Matrix3f covariance; // create covariance matrix
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance); // compute covariance matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors); // create eigen solver
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors(); // get eigen vectors
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); // create 3rd column

    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity()); // create projection transform
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();  // transpose eigen vectors
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>()); // compute projection transform

    pcl::PointCloud<PointT> cloudPointsProjected;  // create projected cloud
    pcl::transformPointCloud(*cluster, cloudPointsProjected, projectionTransform);  // transform cloud
    pcl::getMinMax3D(cloudPointsProjected, minPoint, maxPoint);  // get min and max points

    box.bboxQuaternion = eigenVectorsPCA;  // set box quaternion
    box.bboxTransform = pcaCentroid.head<3>();  // set box transform
    box.cube_length = maxPoint.x - minPoint.x; // set box length
    box.cube_width = maxPoint.y - minPoint.y;  // set box width
    box.cube_height = maxPoint.z - minPoint.z;  // set box height
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
    std::cout << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
typename std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();  // start timer
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};  // create inliers
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients ()};  // create coefficients
    pcl::SACSegmentation<PointT> seg;  // create segmentation object

    seg.setOptimizeCoefficients (true);  // optional
    seg.setModelType (pcl::SACMODEL_PLANE); // set model type
    seg.setMethodType (pcl::SAC_RANSAC);  // set method type
    seg.setMaxIterations (maxIterations);  // set max iterations
    seg.setDistanceThreshold (distanceThreshold);  // set distance threshold

    seg.setInputCloud (cloud);  // set input cloud
    seg.segment (*inliers, *coefficients);  // segment cloud
    if (inliers->indices.size () == 0) // check if no inliers were found
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    auto endTime = std::chrono::steady_clock::now();  // end timer
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);  // calculate elapsed time
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;

}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {
    std::vector<boost::filesystem::path> paths(
            boost::filesystem::directory_iterator{dataPath},
            boost::filesystem::directory_iterator{}
    );

    sort(paths.begin(), paths.end()); // sort files in ascending order so playback is chronological

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
            inliers.insert(rand() % (cloud->points.size()));  // insert random point

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

        // Calculate coefficients for line
        float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        float d = -(a * x1 + b * y1 + c * z1);
        const float norm = sqrt(a * a + b * b + c * c);

        for (int index = 0; index < cloud->points.size(); index++) {
            if (inliers.count(index) > 0)  // if point is already in inliers set, skip
                continue;

            PointT point = cloud->points[index];  // get point
            float x4 = point.x;
            float y4 = point.y;
            float z4 = point.z;

            // Measure distance between every point and fitted line
            float dist = fabs(a * x4 + b * y4 + c * z4 + d) / norm;

            if (dist <= distanceThreshold)  // If distance is smaller than threshold count it as inlier
                inliers.insert(index);
        }

        // Return indices of inliers from fitted line with most inliers
        if (inliers.size() > inliersResult.size()) inliersResult = inliers;
    }

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());  // create inlier cloud
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());  // create outlier cloud

    // Separate inliers and outliers
    for (int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))  // if point is in inliers set, add to inliers cloud
            cloudInliers->points.push_back(point);
        else  // else add to outliers cloud
            cloudOutliers->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

static void recursiveClustering(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, TreeKD* tree, float distanceTol)
{
    processed[indice] = true;  // set point as processed
    cluster.push_back(indice);  // add point to cluster
    std::vector<int> nearest = tree->search(points[indice], distanceTol);  // get nearest points

    // iterate through nearest points
    for (int id : nearest)
    {
        if (!processed[id])  // if point has not been processed, recursively call function
            recursiveClustering(id, points, cluster, processed, tree, distanceTol);
    }
}

template <typename PointT>
typename std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering3D(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    std::vector<std::vector<int>> clusters;  // vector of clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> cloudClusters;  // vector of cloud clusters
    std::vector<bool> processed(cloud->points.size(), false);  // vector of processed points
    std::vector<std::vector<float>> distances;  // vector of distances

    auto *tree = new TreeKD;  // create tree

    for (int i = 0; i < cloud->points.size(); i++) {
        // add point to tree
        tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
        // add point to distances vector
        distances.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    }

    for (int i = 0; i < distances.size(); i++) {
        if (processed[i])  // if point has been processed, skip
            continue;

        std::vector<int> cluster;

        // recursively cluster points
        recursiveClustering(i, distances,  cluster, processed, tree, clusterTolerance);
        clusters.push_back(cluster);
    }

    delete tree;

    // iterate through clusters
    for (const std::vector<int> &cluster: clusters) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        // iterate through points in cluster
        for (int indice: cluster)
            // add point to cloud cluster
            cloudCluster->points.push_back(cloud->points[indice]);

        cloudCluster->width = cloudCluster->points.size();  // set width
        cloudCluster->height = 1;  // set height
        cloudCluster->is_dense = true;  // set density

        // if cluster is within size range, add to cloud clusters
        if (cloudCluster->points.size() >= minSize && cloudCluster->points.size() <= maxSize)
            cloudClusters.push_back(cloudCluster);  // add cloud cluster to vector of cloud clusters
    }

    return cloudClusters;
}