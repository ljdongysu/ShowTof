cmake_minimum_required(VERSION 3.20)
project(ShowTof)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VTK 7 REQUIRED)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${OpenCV_INCLUDE_DIRS})
include(find_yaml.cmake)
#add_subdirectory(cloud_by_pcl)
#add_subdirectory(cloud_by_pangolin)

#add_library(GetCamera SHARED utils/utils.cpp utils/utils.h)
#target_link_libraries(GetCamera ${OpenCV_LIBS} ${VTK_LIBRARIES} ${PCL_LIBRARIES} -ldl)

include_directories(${VTK_INCLUDE_DIRS})
link_directories(${VTK_LIBRARY_DIRS})

add_executable(ShowTof main.cpp SpeckleFilter3d.h SpeckleFileter3d.cpp)

target_link_libraries (ShowTof ${VTK_LIBRARIES} ${PCL_LIBRARIES} ${YAMLCPP_LIBRARY}  ${OpenCV_LIBS})
#
#add_executable(cloud_point_read_show read_point_cloud_show.cpp)
#
#target_link_libraries (cloud_point_read_show ${VTK_LIBRARIES} ${PCL_LIBRARIES} GetCamera)