
project(highSpeedRailway)
cmake_minimum_required(VERSION 3.16)
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")


list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
include_directories( "/usr/include/eigen3" )
# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# 寻找G2O
find_package( G2O REQUIRED )
include_directories(  ${G2O_INCLUDE_DIRS}   )
# CSparse
find_package( CSparse REQUIRED)
include_directories( ${CSPARSE_INCLUDE_DIR} )
include_directories(${CSPARSE_INCLUDE_DIR})

include_directories(./highSpeedRailwayLib)

link_libraries(${OpenCV_LIBS}
              ${Sophus_LIBRARIES}
              g2o_core g2o_stuff g2o_types_sba  
              g2o_types_slam3d 
              g2o_solver_csparse g2o_csparse_extension
              cholmod g2o_solver_cholmod cxsparse)
add_subdirectory(highSpeedRailwayLib)

add_executable(highSpeedRailway curve_fit.cpp)
target_link_libraries(highSpeedRailway highSpeedRailwayData)
set_target_properties(highSpeedRailway PROPERTIES OUTPUT_NAME highSpeedRailway${EXE_POSTFIX})

project(kalman)
add_executable(kalman kalman.cpp ./kalman/kalman_filter.cpp ./kalman/kalman_filter.h)
include_directories(kalman)
include_directories(./highSpeedRailwayLib)
target_link_libraries(kalman highSpeedRailwayData)
set_target_properties(kalman PROPERTIES OUTPUT_NAME kalman${EXE_POSTFIX})
