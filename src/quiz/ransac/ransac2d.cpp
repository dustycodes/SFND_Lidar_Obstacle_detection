/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("/home/dusty/Work/DustyCodes/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit line
        int point1Index = std::rand() % cloud->points.size();
        int point2Index = std::rand() % cloud->points.size();
        auto point1 = cloud->points[point1Index];
        auto point2 = cloud->points[point2Index];
        // (y1-y2)x+(x2-x1)y+(x1∗y2-x2∗y1
        // Ax+By+C=0
        auto a = point1.y - point2.y;
        auto b = point2.x - point1.x;
        auto c = point1.x * point2.y - point2.x * point1.y;

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
            // d=∣Ax+By+C∣/sqrt(A^2+B^2)
            auto p = cloud->points[index];
            auto distance = std::fabs(a * p.x + b * p.y + c) / sqrt(a * a + b * b);

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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    for (int i = 0; i < maxIterations; ++i)
    {
        // Randomly sample subset and fit line
        int point1Index = std::rand() % cloud->points.size();
        int point2Index = std::rand() % cloud->points.size();
        int point3Index = std::rand() % cloud->points.size();
        auto point1 = cloud->points[point1Index];
        auto point2 = cloud->points[point2Index];
        auto point3 = cloud->points[point3Index];

        pcl::PointXYZ v1 = {point2.x - point1.x, point2.y - point1.y, point2.z - point1.z};
        pcl::PointXYZ v2 = {point3.x - point1.x, point3.y - point1.y, point3.z - point1.z};

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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,0,1));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
