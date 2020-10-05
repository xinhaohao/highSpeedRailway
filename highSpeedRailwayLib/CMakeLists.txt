cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 14)


FILE(GLOB ALL_SOURCES "*.cpp")
FILE (GLOB ALL_INCLUDES "*.hpp" "*.h")
SET(ALL_FILES ${ALL_SOURCES} ${ALL_INCLUDES})

add_library(highSpeedRailwayData ${ALL_FILES})
