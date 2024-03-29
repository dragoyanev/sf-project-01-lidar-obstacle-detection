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

    // TODO:: OK Fill in the function to do voxel grid point reduction and region based filtering

    typename pcl::PointCloud<PointT>::Ptr filteredCloud {new pcl::PointCloud<PointT>};

    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr boxRegion {new pcl::PointCloud<PointT>};

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(filteredCloud);
    region.filter(*boxRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(boxRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    // Create filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(boxRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*boxRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return boxRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO:OK Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud {new pcl::PointCloud<PointT>};
    typename pcl::PointCloud<PointT>::Ptr planeCloud {new pcl::PointCloud<PointT>};

    // Create filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*planeCloud);

    // Extract the obstacles
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    // Create segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional optimizitation
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneMy(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr  inliersCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr outliersCloud(new pcl::PointCloud<PointT>());

    std::unordered_set<int> inliers = Ransac3d(cloud, maxIterations, distanceThreshold);

    if (inliers.size() == 0)
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliers.count(index))
            inliersCloud->points.push_back(point);
        else
            outliersCloud->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(outliersCloud, inliersCloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: OK Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for (pcl::PointIndices getIndices : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points.at(index));

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
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::ClusteringMy(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;

    // TODO:: OK Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); i++) {
        PointT p1 = cloud->points.at(i);
        points.push_back({p1.x, p1.y, p1.z});
    }

    KdTree* tree = new KdTree;

    for (int i = 0; i < cloud->points.size(); i++)
        tree->insert(points[i], i);

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);

    for(std::vector<int> cluster : clusters) {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster) {
            PointT point;
            point.x = points[indice][0];
            point.y = points[indice][1];
            point.z = points[indice][2];
            clusterCloud->points.push_back(point);
        }

        if (clusterCloud->points.size() > minSize && clusterCloud->points.size() < maxSize)
            clusterClouds.push_back(clusterCloud);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusterClouds.size() << " clusters" << std::endl;

    return clusterClouds;
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
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters

    // Compute principal direction
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigenSolver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigDx = eigenSolver.eigenvectors();
    eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));


    // move the points to the that reference frame
    Eigen::Matrix4f p2w(Eigen::Matrix4f::Identity());
    p2w.block<3,3>(0,0) = eigDx.transpose();
    p2w.block<3,1>(0,3) = -1.f * (p2w.block<3,3>(0,0) * centroid.head<3>());
    pcl::PointCloud<PointT> cPoints;
    pcl::transformPointCloud(*cluster, cPoints, p2w);

    PointT min_pt, max_pt;
    pcl::getMinMax3D(cPoints, min_pt, max_pt);
    const Eigen::Vector3f mean_diag = 0.5f*(max_pt.getVector3fMap() + min_pt.getVector3fMap());

    // final transform
    const Eigen::Quaternionf qfinal(eigDx);
    const Eigen::Vector3f tfinal = eigDx*mean_diag + centroid.head<3>();

    BoxQ box;
    box.bboxQuaternion = qfinal;
    box.bboxTransform = tfinal;
    box.cube_length = max_pt.x - min_pt.x;
    box.cube_width = max_pt.y - min_pt.y;
    box.cube_height = max_pt.z - min_pt.z;

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
std::unordered_set<int> ProcessPointClouds<PointT>::Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // Measure the time
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // For max iterations
    while (maxIterations--) {
        // Randomly pic three points
        std::unordered_set<int> inliersResultTmp;

        while (inliersResultTmp.size() < 3)
            inliersResultTmp.insert(rand() % cloud->points.size());

        auto itr = inliersResultTmp.begin();
        PointT p1 = cloud->points.at(*itr);
        itr++;
        PointT p2 = cloud->points.at(*itr);
        itr++;
        PointT p3 = cloud->points.at(*itr);

        PointT v1;
        PointT v2;
        v1.x = p2.x - p1.x;
        v1.y = p2.y - p1.y;
        v1.z = p2.z - p1.z;
        v2.x = p3.x - p1.x;
        v2.y = p3.y - p1.y;
        v2.z = p3.z - p1.z;
        PointT v1xv2;
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

            PointT point =  cloud->points.at(j);

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

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster,
                   std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
    processed[indice] = true;
    cluster.push_back(indice);

    std::vector<int> nearest = tree->search(points[indice], distanceTol);

    for (int id : nearest) {
        if (!processed[id])
            clusterHelper(id , points, cluster, processed, tree, distanceTol);
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
    // TODO: OK Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);

    int i = 0;
    while (i < points.size()) {
        if (processed[i]) {
            i++;
            continue;
        }

        std::vector<int> cluster;
        clusterHelper(i, points, cluster, processed, tree, distanceTol);
        clusters.push_back(cluster);
        i++;

    }

    return clusters;
}
