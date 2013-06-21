cmake_minimum_required(VERSION 2.4.6)

set( CMAKE_BUILD_TYPE "Debug" )

find_package(OpenCV REQUIRED)

#set the default path for built executables to the "bin" directory
set( EXECUTABLE_OUTPUT_PATH ${GNG_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set( LIBRARY_OUTPUT_PATH ${GNG_DIR}/lib )

#...: GTest :...................................#
#enable_testing()
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

include_directories(
  /opt/ros/fuerte/include/
  /opt/ros/fuerte/include/opencv
  /opt/ros/fuerte/include/opencv2
  /opt/ros/fuerte/include/opencv2/core
  
  ${GNG_DIR}/include
  ${JOHNSON_APSP_DIR}/include
)

link_directories(
  /usr/local/lib
  /opt/ros/fuerte/stacks/vision_opencv/opencv2/opencv/lib
  /opt/ros/fuerte/lib
  ${GNG_DIR}/lib
  ${JOHNSON_APSP_DIR}/lib
)

link_libraries(	
  ${OpenCV_LIBS}
  johnsonAPSP
)

add_library(toolBoxNG src/CToolBoxNG.cpp)

add_library(neuralGasEdge src/CNeuralGasEdge.cpp src/CNeuralGasNode.cpp)

add_library(neuralGasNode src/CNeuralGasNode.cpp src/CNeuralGasEdge.cpp)

add_library(neuralGas src/CToolBoxNG.cpp src/CNeuralGas.cpp  src/CNeuralGasNode.cpp src/CNeuralGasEdge.cpp src/CMds.cpp)

#add_library(mds src/CMds.cpp)
add_library(pcaCv SHARED src/CPcaCv.cpp)

add_library(neuralGasClassifier SHARED src/CMds.cpp src/CNeuralGasClassifier src/CNeuralGas.cpp  src/CNeuralGasNode.cpp src/CNeuralGasEdge.cpp)

#...: Unit Testing :............................#
#add_executable( tool_box_ng_unit_tests test/CToolBoxNG_test.cpp )
#target_link_libraries( tool_box_ng_unit_tests gtest )