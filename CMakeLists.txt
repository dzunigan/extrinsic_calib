project(extrinsic_calib)

cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native -march=native")

#Get custom build paths
get_filename_component(MRPT_PATHS "../MRPT/build" ABSOLUTE)
set(MRPT_DIR ${MRPT_PATHS} CACHE PATH "Preferred MRPT config path")

find_package(MRPT REQUIRED COMPONENTS gui obs CONFIG PATHS ${MRPT_PATHS})
find_package(Boost REQUIRED COMPONENTS regex filesystem system)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include ${MRPT_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB HEDAER_FILES include/*.hpp)
file(GLOB SRC_FILES src/*.cpp)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${MRPT_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})