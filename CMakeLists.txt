cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set( GNG_DIR ${PROJECT_SOURCE_DIR}/common/growing_neural_gas )
set( JOHNSON_APSP_DIR  ${PROJECT_SOURCE_DIR}/common/johnsonAPSP )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
)

#:.... Libraries ...................................................................................

#:..... Create GNG Library
add_subdirectory( common/growing_neural_gas )

#..: ObjectCandidateExtraction
rosbuild_add_library( ObjectCandidateExtraction ros/src/ObjectCandidateExtraction.cpp )

#..: PlatformExtraction
rosbuild_add_library( PlatformExtraction ros/src/PlatformExtraction.cpp )


#compile the class(es) into a library
#rosbuild_add_library(my_functionality_lib common/src/my_functional_class.cpp)

#create an executable for your ros node
#rosbuild_add_executable(object_reconstruction ros/nodes/object_reconstruction.cpp)

#link a executable against the previously created library
#target_link_libraries(object_reconstruction my_functionality_lib)