cmake_minimum_required(VERSION 2.4.6)

set (CMAKE_BUILD_TYPE "Debug")
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

link_directories(
  ${PROJECT_SOURCE_DIR}/lib
)






add_library(johnsonAPSP SHARED
	src/apsp_graph.cpp 
	src/apsp_node.cpp 
	src/edge.cpp
	src/graph.cpp 
	src/node.cpp
	src/prio_queue.cpp
)

