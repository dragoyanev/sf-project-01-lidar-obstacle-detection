# Project 01 Lidar Obstacle Detection Part Of Sensor Fusion Self-Driving Car Course 

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

This project is part of [Udacity's Sensor Fusion Engineer Nanodegree](https://udacity.com) and is based on the template project provided by the course.
### Project specifications

Its used a sequence of Lidar images (PCD files) demonstrating the implemented techniques.

**Lidar** sensing gives us high resolution data by sending out thousands of laser signals. These lasers bounce off objects, returning to the sensor where we can then determine how far away objects are by timing how long it takes for the signal to return. Each laser ray is in the infrared spectrum, and is sent out at many different angles, usually in a 360 degree range. Lidar sensors gives us very high accurate models for the world around us in 3D.

1. Bounding boxes enclose appropriate objects: bounding boxes enclose vehicles, and the pole on the right side of the vehicle. There is one box per detected object.
2. Objects are consistently detected across frames in the video: most bounding boxes can be followed through the lidar stream, and major objects don't lose or gain bounding boxes in the middle of the lidar stream.
3. Segmentation is implemented in the project: the code used for segmentation uses the 3D RANSAC algorithm developed in the course lesson. 
4. Clustering is implemented in the project: the code used for clustering uses the Euclidean clustering algorithm along with the KD-Tree developed in the course lesson.

## Installation

### Ubuntu 

https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for

### Windows 

http://www.pointclouds.org/downloads/windows.html

### MAC

#### Install via Homebrew
1. install [homebrew](https://brew.sh/)
2. update homebrew 
	```bash
	$> brew update
	```
3. add  homebrew science [tap](https://docs.brew.sh/Taps) 
	```bash
	$> brew tap brewsci/science
	```
4. view pcl install options
	```bash
	$> brew options pcl
	```
5. install PCL 
	```bash
	$> brew install pcl
	```

#### Prebuilt Binaries via Universal Installer
http://www.pointclouds.org/downloads/macosx.html  
NOTE: very old version 

#### Build from Source

[PCL Source Github](https://github.com/PointCloudLibrary/pcl)

[PCL Mac Compilation Docs](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_macosx.php)
