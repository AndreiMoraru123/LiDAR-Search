# LiDAR-Road-Segmentation-and-Obstacle-Detection
A 3D LiDAR simulation for drivable area segmentaiton, vehicle and pedestrian clustering. The simulation works on real pcd data with the help of PCL and renders bounding boxes for the detections

## Raw data

![block](https://user-images.githubusercontent.com/81184255/194830479-2a31b3af-14c4-4496-95cb-99825f5da65c.gif)

## Filtering

```cpp
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
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

    return cloudRegion;
}
```
![blockFilt](https://user-images.githubusercontent.com/81184255/194831116-cb8e3edc-7a52-47b8-b36c-29981ec5b96e.gif)

## Segmentation using Rnadom Sample Consensus

```cpp
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    while (maxIterations--) {
        // Randomly sample subset and fit a plane
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
```

![blockSeg](https://user-images.githubusercontent.com/81184255/194831441-8d3e0e5c-920c-4f0c-8866-6915551d3326.gif)

## K-dim Tree data structure for clustering

```cpp
struct Node{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct TreeKD
{
    Node* root;

    TreeKD()
            : root(NULL)
    {}

    ~TreeKD()
    {
        delete root;
    }

    void insertHelper(Node** node, int depth, std::vector<float> point, int id)
    {
        if (*node == NULL)
        {
            *node = new Node(point, id);
        }
        else
        {
            int cd = depth % 2;
            if (point[cd] < ((*node)->point[cd]))
            {
                insertHelper(&((*node)->left), depth + 1, point, id);
            }
            else
            {
                insertHelper(&((*node)->right), depth + 1, point, id);
            }
        }
    }

    void insert(std::vector<float> point, int id)
    {
        Node** node = &root;
        int depth = 0;
        while (*node != NULL)
        {
            int cd = depth % 2;
            if (point[cd] < (*node)->point[cd])
                node = &(*node)->left;
            else
                node = &(*node)->right;
            depth++;
        }
        *node = new Node(point, id);
    }

    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        std::vector<int> ids;
        searchHelper(target, distanceTol, root, 0, ids);
        return ids;
    }

    void searchHelper(std::vector<float> target, float distanceTol, Node* node, int depth, std::vector<int>& ids)
    {
        if (node != NULL)
        {
            if ((node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol)) &&
                (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)))
            {
                float distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) +
                                      (node->point[1] - target[1]) * (node->point[1] - target[1]));
                if (distance <= distanceTol)
                    ids.push_back(node->id);
            }

            if ((target[depth % 2] - distanceTol) < node->point[depth % 2])
                searchHelper(target, distanceTol, node->left, depth + 1, ids);
            if ((target[depth % 2] + distanceTol) > node->point[depth % 2])
                searchHelper(target, distanceTol, node->right, depth + 1, ids);
        }
    }
};
```

provides support for K-means clustering

```cpp
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering3D(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

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
```

by using an Euclidean distance based search

```cpp
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
```
![blockClust](https://user-images.githubusercontent.com/81184255/194832954-94afddb8-0acf-4baf-868c-c27004e2d8a4.gif)

## Bounding Box encapsulation for detected objects

```cpp
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
```

that use a 3D Box data structure

```cpp
struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};
```
![blockBox](https://user-images.githubusercontent.com/81184255/194833573-ca4f09c7-9c85-45c2-ad1c-21a89a99b38a.gif)

## Eigen provides functions for rendering the pose of the boxes using quaternions as the represatation of the rotation

```cpp
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
```
using a customized data structure for the boxes

```cpp
struct BoxQ
{
    Eigen::Vector3f bboxTransform;
    Eigen::Quaternionf bboxQuaternion;
    float cube_length;
    float cube_width;
    float cube_height;
};
```

![blockQ](https://user-images.githubusercontent.com/81184255/194833832-e64f160c-f7ca-4044-b3fd-8c4e63a566dd.gif)
