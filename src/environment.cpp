/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
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

static const double LIDAR_GROUND_SLOPE_ANGLE = 0.0;
static const int POINT_PROCESSOR_MAX_ITERATIONS = 100;
static const double POINT_PROCESSOR_DISTANCE_THRESHOLD = 0.2;

static const double POINT_PROCESSOR_CLUSTERING_DISTANCE_THRESHOLD = 1.0;
static const int POINT_PROCESSOR_CLUSTERING_MIN_CLUSTER_POINTS = 3;
static const int POINT_PROCESSOR_CLUSTERING_MAX_CLUSTER_POINTS = 30;

static const double POINT_PROCESSOR_I_VOXEL_SIZE = 0.2;
static const double POINT_PROCESSOR_I_CLUSTERING_DISTANCE_THRESHOLD = 0.5;
static const int POINT_PROCESSOR_I_CLUSTERING_MIN_CLUSTER_POINTS = 10;
static const int POINT_PROCESSOR_I_CLUSTERING_MAX_CLUSTER_POINTS = 3000;

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI> *pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    bool renderClusterEn = true;
    bool renderBoxEn = true;
    bool renderBoxQEn = false;

    Eigen::Vector4f minPoint = Eigen::Vector4f(-10, -5, -2, 1);
    Eigen::Vector4f maxPoint = Eigen::Vector4f(30, 7, 1, 1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, POINT_PROCESSOR_I_VOXEL_SIZE, minPoint, maxPoint);

//    renderPointCloud(viewer, inputCloud, "inputCloud");
//    renderPointCloud(viewer, filterCloud, "filterCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneMy(filterCloud, POINT_PROCESSOR_MAX_ITERATIONS, POINT_PROCESSOR_DISTANCE_THRESHOLD);
//    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    // Draw roof crop box
    Box box;
    box.x_min = -1.5;
    box.x_max = 2.6;
    box.y_min = -1.7;
    box.y_max = 1.7;
    box.z_min = -1;
    box.z_max = -0.4;
    renderBox(viewer, box, 555, Color(1, 0, 1));


    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, POINT_PROCESSOR_I_CLUSTERING_DISTANCE_THRESHOLD, POINT_PROCESSOR_I_CLUSTERING_MIN_CLUSTER_POINTS, POINT_PROCESSOR_I_CLUSTERING_MAX_CLUSTER_POINTS);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 1),
                                 Color(1, 0, 0), Color(0, 1, 1),
                                 Color(1, 1, 1), Color(0, 0, 1),};

    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
        if (renderClusterEn) {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        }

        if (renderBoxEn) {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if (renderBoxQEn) {
            // If we want to debug at the same time both render boxes we need unique string Id for the render box
            // Thets why I use some offset here as patch
            int sameBoxesOffset = 0;
            if (renderBoxEn == true)
                sameBoxesOffset = 100;

            BoxQ boxq = pointProcessorI->BoundingBoxQ(cluster);
            renderBox(viewer, boxq, clusterId + sameBoxesOffset);
        }

        ++clusterId;
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderClusterEn = true;
    bool renderBoxEn = true;
    bool renderBoxQEn = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO::OK Create lidar sensor
    Lidar *lidar = new Lidar(cars, LIDAR_GROUND_SLOPE_ANGLE);

    cout <<"Lidar Pos x="<< lidar->position.x << " y="<< lidar->position.y <<" z="<< lidar->position.z <<endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarScan = lidar->scan();
//    renderRays(viewer, lidar->position, lidarScan);
//    renderPointCloud(viewer, lidarScan, "PointCloud", Color(1, 1, 1));

    // TODO::OK Create point processor
    ProcessPointClouds<pcl::PointXYZ> *pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidarScan, POINT_PROCESSOR_MAX_ITERATIONS, POINT_PROCESSOR_DISTANCE_THRESHOLD);
//    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
//    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, POINT_PROCESSOR_CLUSTERING_DISTANCE_THRESHOLD, POINT_PROCESSOR_CLUSTERING_MIN_CLUSTER_POINTS, POINT_PROCESSOR_CLUSTERING_MAX_CLUSTER_POINTS);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
        if (renderClusterEn) {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId), colors[clusterId % colors.size()]);
        }

        if (renderBoxEn) {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
        }

        if (renderBoxQEn) {
            // If we want to debug at the same time both render boxes we need unique string Id for the render box
            // Thets why I use some offset here as patch
            int sameBoxesOffset = 0;
            if (renderBoxEn == true)
                sameBoxesOffset = 100;

            BoxQ boxq = pointProcessor->BoundingBoxQ(cluster);
            renderBox(viewer, boxq, clusterId + sameBoxesOffset);
        }

        ++clusterId;
    }

}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
//    simpleHighway(viewer);

    // Create point processor
    ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
//    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
//    cityBlock(viewer, pointProcessorI, inputCloudI);

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run odstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    } 
}
