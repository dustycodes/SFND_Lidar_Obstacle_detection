// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxelGrid;
    pcl::CropBox<PointT> region(true);
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr regionCloud(new pcl::PointCloud<PointT>());
    std::vector<int> indices;

    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*filteredCloud);

    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*regionCloud);

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(regionCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(regionCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*regionCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return regionCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());

    for (auto index : inliers->indices)
    {
        plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    bool useCustomImpl = true;

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;

    if (useCustomImpl)
    {
        auto inliers = RansacPlane(cloud, maxIterations, distanceThreshold);

        if (inliers.size() <= 0)
        {
            std::cout << "Could not estimate" << std::endl;
        }

        typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
        typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

        for (int index = 0; index < cloud->points.size(); index++)
        {
            PointT point = cloud->points[index];
            if (inliers.count(index))
                cloudInliers->points.push_back(point);
            else
                cloudOutliers->points.push_back(point);
        }
        segResult.first = cloudOutliers;
        segResult.second = cloudInliers;
    }
    else
    {
	    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);

        pcl::SACSegmentation<PointT> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceThreshold);
        seg.setInputCloud(cloud);
        seg.segment(*inliers2, *coefficients);

        if (inliers2->indices.size() <= 0)
        {
            std::cout << "Could not estimate" << std::endl;
        }

        segResult = SeparateClouds(inliers2, cloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    bool useCustomImpl = true;
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction

    if (useCustomImpl)
    {
        std::vector<int> clusterIndices;
        KdTree* tree = new KdTree();
        std::vector<std::vector<float>> points;

        for (int i=0; i < cloud->points.size(); i++)
        {
            tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
            points.push_back({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
        }

        auto ecs = euclideanCluster(points, tree, clusterTolerance);
        ecs.erase(std::remove_if(
                ecs.begin(), ecs.end(),
                [minSize, maxSize](std::vector<int>* cluster) -> bool
                {
                    return cluster->size() > maxSize || cluster->size() < minSize;
                }));

        int clusterId = 0;
        for(const std::vector<int>& c : ecs)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
            for(int index : c)
            {
                clusterCloud->points.push_back(cloud->points[index]);
            }
            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;
            ++clusterId;
        }
    }
    else
    {
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(clusterTolerance); // 2cm
        ec.setMinClusterSize(minSize);
        ec.setMaxClusterSize(maxSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        for (const auto& clusterIndex : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>);
            for (auto pit = clusterIndex.indices.begin(); pit != clusterIndex.indices.end(); ++pit)
            {
                clusterCloud->points.push_back(cloud->points[*pit]);
            }
            clusterCloud->width = clusterCloud->points.size();
            clusterCloud->height = 1;
            clusterCloud->is_dense = true;

            clusters.push_back(clusterCloud);
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    // TODO do PCA to get a fit boudning box on cluster
//    pcl::getMinMax3D(*cluster, minPoint, maxPoint);
//
    BoxQ box;
//    box.x_min = minPoint.x;
//    box.y_min = minPoint.y;
//    box.z_min = minPoint.z;
//    box.x_max = maxPoint.x;
//    box.y_max = maxPoint.y;
//    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    if (cloud->points.size() <= 0)
    {
        std::cerr << "There was no points in the cloud" << std::endl;
        return inliersResult;
    }

    // For max iterations
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit line
        std::cout << "Cloud points " << cloud->points.size() << std::endl;
        int point1Index = std::rand() % cloud->points.size();
        int point2Index = std::rand() % cloud->points.size();
        int point3Index = std::rand() % cloud->points.size();
        auto point1 = cloud->points[point1Index];
        auto point2 = cloud->points[point2Index];
        auto point3 = cloud->points[point3Index];

        // find normal vector by taking cross product
        // (y1-y2)x+(x2-x1)y+(x1∗y2-x2∗y1
        // Ax+By+C=0
        auto a = (point2.y - point1.y) * (point3.z - point1.z) - (point2.z - point1.z) * (point3.y - point1.y);
        auto b = (point2.z - point1.z) * (point3.x - point1.x) - (point2.x - point1.x) * (point3.z - point1.z);
        auto c = (point2.x - point1.x) * (point3.y - point1.y) - (point2.y - point1.y) * (point3.x - point1.x);
        auto d = -1 * (a * point1.x + b * point1.y + c * point1.z);
        // D=−(ix1+jy1+kz1)

        std::unordered_set<int> inliers;
        inliers.insert(point1Index);
        inliers.insert(point2Index);

        for (int index = 0; index < cloud->points.size(); ++index)
        {
            if (index == point1Index || index == point2Index)
            {
                continue;
            }

            // Measure distance between every point and fitted line
            // Ax+By+Cz+D=0
            // d=∣A∗x+B∗y+C∗z+D∣/sqrt(A^2+B^2+C^2).
            auto p = cloud->points[index];
            auto distance = std::fabs(a * p.x + b * p.y + c * p.z + d) / sqrt(a * a + b * b + c * c);

            // If distance is smaller than threshold count it as inlier
            if (distance <= distanceTol)
            {
                inliers.insert(index);
            }
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }
    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>>& points, KdTree *tree,
                                           float distanceTol, std::vector<bool>& processed, std::vector<int>& cluster)
{
    processed[index] = true;
    cluster.push_back(index);

    auto nn = tree->search(points[index], distanceTol);

    for(int id : nn)
    {
        if (not processed[id])
        {
            proximity(id, points, tree, distanceTol, processed, cluster);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree *tree,
                                             float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    // Keep track of processed points
    std::vector<bool> processed(points.size(), false);

    // Iterate through each point
    for (int i = 0; i < points.size(); ++i)
    {
        // If point has not been processed
        if (processed[i])
        {
            continue;
        }

        // Create cluster
        std::vector<int> cluster;
        proximity(i, points, tree, distanceTol, processed, cluster);

        clusters.push_back(cluster);
    }

    return clusters;
}
