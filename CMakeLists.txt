cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#.....: Compiler Options :.........................................................................:

#.....: gnu++0x
if(CMAKE_COMPILER_IS_GNUCXX)
    execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
    if(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        add_definitions(-std=gnu++0x)
    else(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
        message(SEND_ERROR "You need GCC version 4.6 or greater to compile this package.")
    endif(GCC_VERSION VERSION_GREATER 4.6 OR GCC_VERSION VERSION_EQUAL 4.6)
endif(CMAKE_COMPILER_IS_GNUCXX)

rosbuild_gensrv()

find_package( PCL 1.7 REQUIRED )
find_package( VTK REQUIRED )

set( GNG_DIR ${PROJECT_SOURCE_DIR}/common/growing_neural_gas )
set( JOHNSON_APSP_DIR  ${PROJECT_SOURCE_DIR}/common/johnsonAPSP )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
   ${PROJECT_SOURCE_DIR}/srv_gen/cpp/include
   ${PROJECT_SOURCE_DIR}/common/include
   ${PROJECT_SOURCE_DIR}/ros/include
   ${PCL_INCLUDE_DIRS}
)

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

#.....: Libraries .................................................................................:

#.....: Create GNG Library
#add_subdirectory( common/GrowingNeuralGas )

#.....: PointCloudAccumulation
rosbuild_add_library( PointCloudAccumulator ros/src/PointCloudAccumulator.cpp )
target_link_libraries( PointCloudAccumulator ${PCL_LIBRARIES} )

#.....: Object Candidate Extractor 
rosbuild_add_library( ObjectCandidateExtractor ros/src/ObjectCandidateExtractor.cpp )
target_link_libraries( ObjectCandidateExtractor ${PCL_LIBRARIES} )

#.....: HelperFunctions
rosbuild_add_library( HelperFunctions ros/src/HelperFunctions.cpp )
target_link_libraries( HelperFunctions ${PCL_LIBRARIES} )

#.....: Occlusion Repair
rosbuild_add_library( OcclusionRepair ros/src/OcclusionRepair.cpp )
target_link_libraries( OcclusionRepair ${VTK_LIBRARIES} )

#......: ROS NODE :................................................................................:
rosbuild_add_executable( hbrs_object_reconstruction ros/nodes/object_reconstruction.cpp )
target_link_libraries( hbrs_object_reconstruction PointCloudAccumulator )
target_link_libraries( hbrs_object_reconstruction ObjectCandidateExtractor )
target_link_libraries( hbrs_object_reconstruction HelperFunctions )
target_link_libraries( hbrs_object_reconstruction OcclusionRepair )

#.....: Generate Documentation :...................................................................:
find_package(Doxygen REQUIRED)

set( DOXYGEN_CONFIGURED_INPUT ${PROJECT_SOURCE_DIR}/Doxyfile )

add_custom_target( Documentation ALL
                   COMMAND ${CMAKE_COMMAND} -E echo "Building Documentation..."
                   COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_CONFIGURED_INPUT}
                   COMMAND ${CMAKE_COMMAND} -E echo "Done Building Documentation."
                   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})