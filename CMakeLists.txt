cmake_minimum_required(VERSION 3.15)
project(test VERSION "0.0.1")

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread -fPIC -Wno-unused-result  -Wno-sign-compare -Wno-comment -Wno-deprecated-declarations")
set(EIGEN_INCLUDE_DIR       /usr/include/eigen3)

find_package(PCL REQUIRED 1.8  COMPONENT common io visualization)
find_package(OpenCV REQUIRED)

include_directories(
                    ${EIGEN_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    )

add_executable(test test1.cc)
target_link_libraries(test ${PCL_LIBRARIES} boost_system  ${OpenCV_LIBS}) 

add_executable(PointCloudBinToPcd PointCloudBinToPcd.cc)
target_link_libraries(PointCloudBinToPcd ${PCL_LIBRARIES} boost_system boost_program_options  ${OpenCV_LIBS}) 




