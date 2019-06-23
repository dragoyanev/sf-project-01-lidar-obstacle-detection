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
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
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
	
    // TODO:OK Fill in this function
    // For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers

	// For max iterations 
    for (int i = 0; i < maxIterations; i++) {
        // Randomly sample subset and fit line
        int randomIndex = rand() % cloud->points.size();
        pcl::PointXYZ p1 = cloud->points.at(randomIndex);
        int newRandomIndex = rand() % cloud->points.size();
        while (randomIndex == newRandomIndex)
            newRandomIndex = rand() % cloud->points.size();
        pcl::PointXYZ p2 = cloud->points.at(newRandomIndex);

//        std::cout << "Random line: p1[" << randomIndex << "](" << p1.x <<", "<<p1.y<<")";
//        std::cout << " p2[" << newRandomIndex << "](" << p2.x <<", "<<p2.y<<")" << std::endl;

        double a = p1.y - p2.y;
        double b = p2.x - p1.x;
        double c = p1.x * p2.y - p2.x * p1.y;

        std::unordered_set<int> inliersResultTmp;
        // Measure distance between every point and fitted line
        for(int j = 0; j < cloud->points.size(); ++j) {
            pcl::PointXYZ point =  cloud->points.at(j);
            double distance = std::fabs(a*point.x + b*point.y+c)/std::sqrt(a*a + b*b);
            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol) // inlier
                inliersResultTmp.insert(j);
        }
        // Return indicies of inliers from fitted line with most inliers
        if (inliersResult.size() < inliersResultTmp.size())
            inliersResult = inliersResultTmp;
    }

	return inliersResult;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Measure the time
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO:OK Fill in this function
    // For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers

    // For max iterations
    while (maxIterations--) {
        // Randomly pic three points
        std::unordered_set<int> inliersResultTmp;

        while (inliersResultTmp.size() < 3)
            inliersResultTmp.insert(rand() % cloud->points.size());

        auto itr = inliersResultTmp.begin();
        pcl::PointXYZ p1 = cloud->points.at(*itr);
        itr++;
        pcl::PointXYZ p2 = cloud->points.at(*itr);
        itr++;
        pcl::PointXYZ p3 = cloud->points.at(*itr);

//        std::cout << "Random line: p1[" << randomIndex << "](" << p1.x <<", "<<p1.y<<")";
//        std::cout << " p2[" << newRandomIndex << "](" << p2.x <<", "<<p2.y<<")" << std::endl;
        pcl::PointXYZ v1;
        pcl::PointXYZ v2;
        v1.x = p2.x - p1.x;
        v1.y = p2.y - p1.y;
        v1.z = p2.z - p1.z;
        v2.x = p3.x - p1.x;
        v2.y = p3.y - p1.y;
        v2.z = p3.z - p1.z;
        pcl::PointXYZ v1xv2;
        v1xv2.x = v1.y*v2.z - v1.z*v2.y;
        v1xv2.y = v1.z*v2.x - v1.x*v2.z;
        v1xv2.z = v1.x*v2.y - v1.y*v2.x;

        double a = v1xv2.x;
        double b = v1xv2.y;
        double c = v1xv2.z;
        double d = -(a*p1.x + b*p1.y + c*p1.z);


        // Measure distance between every point and fitted line
        for(int j = 0; j < cloud->points.size(); ++j) {

            // This point is already inlier (one of random points)
            if (inliersResultTmp.count(j) > 0)
                continue;

            pcl::PointXYZ point =  cloud->points.at(j);

            double distance = std::fabs(a*point.x + b*point.y + c*point.z + d)/std::sqrt(a*a + b*b + c*c);
            // If distance is smaller than threshold count it as inlier
            if (distance < distanceTol) // inlier
                inliersResultTmp.insert(j);
        }
        // Return indicies of inliers from fitted line with most inliers
        if (inliersResult.size() < inliersResultTmp.size())
            inliersResult = inliersResultTmp;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "RANSAC 3D took " << elapsedTime.count() << " milliseconds" << std::endl;

    return inliersResult;
}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

    // TODO: OK Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
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
