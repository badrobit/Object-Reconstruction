cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#:..... Compiler Options :.........................................................................

#...: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

rosbuild_gensrv()

find_package( PCL 1.4 REQUIRED )

set( GNG_DIR ${PROJECT_SOURCE_DIR}/common/growing_neural_gas )
set( JOHNSON_APSP_DIR  ${PROJECT_SOURCE_DIR}/common/johnsonAPSP )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
   ${PROJECT_SOURCE_DIR}/srv_gen/cpp/include
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
   ${PCL_INCLUDE_DIRS}
)

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#:.... Libraries ...................................................................................

#:..... Create GNG Library
#add_subdirectory( common/growing_neural_gas )

#..: PointCloudAccumulation
rosbuild_add_library( PointCloudAccumulator ros/src/PointCloudAccumulator.cpp )
target_link_libraries( PointCloudAccumulator ${PCL_LIBRARIES} )


#:..... ROS NODE :................................................................................:
rosbuild_add_executable(hbrs_object_reconstruction ros/nodes/object_reconstruction.cpp)
target_link_libraries( hbrs_object_reconstruction PointCloudAccumulator )