// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
#include"kd_tree.h"
// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr afterFiltered(new typename pcl::PointCloud<PointT>);
    typename pcl::VoxelGrid<PointT> downsampleSolver;
    downsampleSolver.setLeafSize(filterRes, filterRes, filterRes);
    downsampleSolver.setInputCloud(cloud);
    downsampleSolver.filter(*afterFiltered);

    typename pcl::PointCloud<PointT>::Ptr afterCroped(new typename pcl::PointCloud<PointT>);
    typename pcl::CropBox<PointT> cropSolver;
    cropSolver.setMin(minPoint);
    cropSolver.setMax(maxPoint);
    cropSolver.setInputCloud(afterFiltered);
    cropSolver.filter(*afterCroped);

    std::vector<int> roofIndex;
    typename pcl::CropBox<PointT> cropRoofSolver(true);
    cropRoofSolver.setMin(Eigen::Vector4f(-3, -3, -3, 1));
    cropRoofSolver.setMax(Eigen::Vector4f(3, 3, 3, 1));
    cropRoofSolver.setInputCloud(afterCroped);
    cropRoofSolver.filter(roofIndex);

    pcl::PointIndices::Ptr indice(new pcl::PointIndices);
    for (auto indexItem : roofIndex)
        indice->indices.push_back(indexItem);
    typename pcl::ExtractIndices<PointT> extractSolver;
    extractSolver.setInputCloud(afterCroped);
    extractSolver.setIndices(indice);
    extractSolver.setNegative(true);
    extractSolver.filter(*afterCroped);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
        << std::endl;

    return afterCroped;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers,
        typename pcl::PointCloud<PointT>::Ptr cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other with
    // segmented plane
    typename pcl::PointCloud<PointT>::Ptr plane(new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacles(new typename pcl::PointCloud<PointT>);
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);

    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
        typename pcl::PointCloud<PointT>::Ptr>
        segResult(plane, obstacles);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
        float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers;

    pcl::ModelCoefficients::Ptr coefficient(new pcl::ModelCoefficients());
    inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficient);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count()
        << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
        typename pcl::PointCloud<PointT>::Ptr>
        segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
    typename pcl::PointCloud<PointT>::Ptr>
    ProcessPointClouds<PointT>::SegmentPlaneMine(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
        float distanceThreshold) {
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;

    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<> ud(0, cloud->points.size() - 1);
    size_t bestInliers = 0;

    for (int i = 0; i < maxIterations; i++) {
        std::unordered_set<int> inliersIterations;

        int pointIndex1 = ud(mt);
        int pointIndex2 = ud(mt);
        int pointIndex3 = ud(mt);
        while (pointIndex1 == pointIndex2)
            pointIndex2 = ud(mt);
        while (pointIndex2 == pointIndex3)
            pointIndex3 = ud(mt);

        float x1 = cloud->points[pointIndex2].x - cloud->points[pointIndex1].x;
        float y1 = cloud->points[pointIndex2].y - cloud->points[pointIndex1].y;
        float z1 = cloud->points[pointIndex2].z - cloud->points[pointIndex1].z;
        float x2 = cloud->points[pointIndex3].x - cloud->points[pointIndex1].x;
        float y2 = cloud->points[pointIndex3].y - cloud->points[pointIndex1].y;
        float z2 = cloud->points[pointIndex3].z - cloud->points[pointIndex1].z;

        float nx = y1 * z2 - z1 * y2;
        float ny = z1 * x2 - x1 * z2;
        float nz = x1 * y2 - y1 * x2;

        float A = nx;
        float B = ny;
        float C = nz;
        float D =
            -(A * cloud->points[pointIndex1].x + B * cloud->points[pointIndex1].y +
                C * cloud->points[pointIndex1].z);
        float sqrta2b2c2 = sqrt(A * A + B * B + C * C)+0.0001f;

        for (int itemIndex = 0; itemIndex < cloud->points.size();itemIndex++) {
            PointT& pointItem = cloud->points[itemIndex];
            if (abs(A * pointItem.x + B * pointItem.y + C * pointItem.z + D) <
                distanceThreshold*sqrta2b2c2) {
                inliersIterations.insert(itemIndex);
            }
        }
        if (inliersIterations.size() > inliersResult.size()) {
            inliersResult = inliersIterations;
        }
    }

    std::cout << "Inliers: " << inliersResult.size() << std::endl;
    std::cout << "Outliers: " << cloud->points.size()-inliersResult.size() << std::endl;

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count()
        << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
        typename pcl::PointCloud<PointT>::Ptr>
        segResult(cloudInliers, cloudOutliers);
    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    typename pcl::search::KdTree<PointT>::Ptr tree(
        new typename pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it =
        cluster_indices.begin();
        it != cluster_indices.end(); ++it) {
        clusters.push_back(
            typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));

        for (std::vector<int>::const_iterator pit = it->indices.begin();
            pit != it->indices.end(); ++pit)
            clusters.back()->points.push_back(cloud->points[*pit]);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count()
        << " milliseconds and found " << clusters.size() << " clusters"
        << std::endl;

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::ClusteringMine(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree *tree = new KdTree;

    for (int i = 0; i < cloud->points.size(); i++)
        tree->insert({ cloud->points[i].x, cloud->points[i].y, cloud->points[i].z },
            i);

    std::vector<std::vector<int>> clusterIndices = euclideanCluster<PointT>(cloud, tree, clusterTolerance);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    for (size_t clusterIndex = 0; clusterIndex < clusterIndices.size(); clusterIndex++) {
        if (clusterIndices[clusterIndex].size() < minSize)
            continue;
        clusters.push_back(
            typename pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>));

        for (size_t pointIndex = 0; pointIndex < clusterIndices[clusterIndex].size(); pointIndex++)
            clusters.back()->points.push_back(cloud->points[clusterIndices[clusterIndex][pointIndex]]);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count()
        << " milliseconds and found " << " clusters"
        << std::endl;

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file
        << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
        << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(
        boost::filesystem::directory_iterator{ dataPath },
        boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}