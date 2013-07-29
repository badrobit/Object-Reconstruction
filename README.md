Occluded Geometry Estimation
==========================

# API 

The Application Programming Interface (API) for this project is available through the projects [GitHub Page](http://bad-robot.github.io/hbrs_object_reconstruction/index.html)

ROS source code for my R&amp;D project on Occluded Object Geometry Estimation

## Requirements
This section will outline all of the things that you will require in order to properly compile and launch the software contained in this package: 

* An OpenNI compatable [RGBD Sensor](http://en.wikipedia.org/wiki/PrimeSense). 
* A C++ Compiler capable of compiling at least [C++0x](http://gcc.gnu.org/projects/cxx0x.html)
* [ROS Fuerte](http://www.ros.org/)
* [PCL 1.6+ (trunk prefered)](http://pointclouds.org/downloads/source.html)

## Downloading

In order to download you simply need to pull and initialize the git repository like you normally due. As we are using extrenally maintained libraries you also need to update the git submodules if this is your first pull you run both commands otherwise run only the second command:

* `git submodule init` - Initalize the submodule
* `git submodule update` - Update from the externally maintained library if any updates are available. 

For any questions about git's submodules please see the [git Book](http://git-scm.com/book/en/Git-Tools-Submodules)

## Complilation
Where this project is a ROS based project you simply need to run rosmake from within the project folder. You can alternatively run this command from anywhere by providing the full ROS package name: 

`$ rosmake hbrs_object_reconstruction`

## Running Software

## Services

The various components of this system are accessible through **ROS Services** which will allow for either internal or external software to call the different parts of the system in the desired order. The following services are offered by the Object Reconstruction system: 

### `/ExtractPlatform` 
This takes in a **PCL::PointCloud** and will look for the single largest planner object that can be found in the provided scene. It will return to you a **PCL::PointCloud** which containts only the points from the original pointcloud that are found inside of determined platform. 

### `/AccumulatePointClouds` 
This takes in a **32-bit Integer** that represents the amount of time that the caller wants the point clouds to be accumulated for. It will then accumulate point clouds from a **RGBD Sensor** and then package them as a `pcl::PointCloud< PointXYZ >` which will be returned to the service caller through the service return message. The pointcloud accumulation happens through incremental registration which is based off of the following [PCL Tutorial](http://pointclouds.org/documentation/tutorials/pairwise_incremental_registration.php#pairwise-incremental-registration). 

`$ rosservice call /AccumulatePointClouds "accumulation_time: 2"`