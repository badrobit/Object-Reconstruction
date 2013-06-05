cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set( GNG_DIR ${PROJECT_SOURCE_DIR}/common/growing_neural_gas )
set( JOHNSON_APSP_DIR  ${PROJECT_SOURCE_DIR}/common/johnsonAPSP )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

include_directories(
   ${PROJECT_SOURCE_DIR}/ros/include
)

#:..... Create GNG Library ........................................................................:
add_subdirectory( common/growing_neural_gas )

#compile the class(es) into a library
#rosbuild_add_library(my_functionality_lib common/src/my_functional_class.cpp)

#create an executable for your ros node
#rosbuild_add_executable(object_reconstruction ros/nodes/object_reconstruction.cpp)

#link a executable against the previously created library
#target_link_libraries(object_reconstruction my_functionality_lib)