cmake_minimum_required(VERSION 2.8)

# find point cloud library
find_package(PCL 1.8.0 REQUIRED)
if(PCL_FOUND)
    message(STATUS "PCL INCLUDE DIRS ${PCL_INCLUDE_DIRS}")
    message(STATUS "PCL LIBS REQUIRED ${PCL_LIBRARIES}")
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
endif()

find_package(OpenCV 2.4 REQUIRED)
if(OpenCV_FOUND)
    message(STATUS "OpenCV INCLUDE DIRS ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV LIBS REQUIRED ${OpenCV_LIBS}")
endif()

find_package(Boost REQUIRED thread system filesystem)


add_executable(depth2cloud depth2cloud.cpp)
target_link_libraries(depth2cloud ${PCL_LIBRARIES} ${Boost_LIBRARIES})
