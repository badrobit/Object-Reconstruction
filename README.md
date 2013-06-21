Occluded Geometry Estimation
==========================

ROS source code for my R&amp;D project on Occluded Object Geometry Estimation

## Services

The various components of this system are accessible through **ROS Services** which will allow for either internal or external software to call the different parts of the system in the desired order. The following services are offered by the Object Reconstruction system: 

* `ExtractPlatform` This takes in a **PCL::PointCloud** and will look for the single largest planner object that can be found in the provided scene. It will return to you a **PCL::PointCloud** which containts only the points from the original pointcloud that are found inside of determined platform. 