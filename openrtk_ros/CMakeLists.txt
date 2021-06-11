cmake_minimum_required(VERSION 2.8.3)
project(openrtk_ros)

############ Choose Release or Debug ##############
set(COMPILE_TYPE Release)
if(COMPILE_TYPE STREQUAL "Release")
  set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_CXX_FLAGS "-std=c++11 ")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
elseif(COMPILE_TYPE STREQUAL "Debug")
  SET(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "-std=c++11 ")
  SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
else()
  message(FATAL_ERROR "COMPILE_TYPE is Error!")
endif()
  message(AUTHOR_WARNING "COMPILE_TYPE = ${COMPILE_TYPE}")
#################################################

find_package(Threads REQUIRED)

####################################################
set(serial_SRCS
${PROJECT_SOURCE_DIR}/src/3rdpart/serial/src/serial.cc
${PROJECT_SOURCE_DIR}/src/3rdpart/serial/include/serial/serial.h
${PROJECT_SOURCE_DIR}/src/3rdpart/serial/include/serial/v8stdint.h
${PROJECT_SOURCE_DIR}/src/3rdpart/serial/src/impl/unix.cc
${PROJECT_SOURCE_DIR}/src/3rdpart/serial/src/impl/list_ports/list_ports_linux.cc
)

set(DRIVER_SRCS
${PROJECT_SOURCE_DIR}/src/driver/driver.cpp
${PROJECT_SOURCE_DIR}/src/rtk/rtk.cpp
)

aux_source_directory(. DIR_SRCS)

catkin_package()
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  openrtk_msg
  std_msgs
  tf
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_BINARY_DIR})#used to find config.h

include_directories(
  ${PROJECT_SOURCE_DIR}/src/3rdpart/serial/include
  ${PROJECT_SOURCE_DIR}/src/driver
  ${PROJECT_SOURCE_DIR}/src/rtk
  ${PROJECT_SOURCE_DIR}/src/util
  ${PROJECT_SOURCE_DIR}/src/protocol
  )

add_executable(driver src/main.cpp ${DRIVER_SRCS}  ${serial_SRCS} ${DIR_SRCS})
target_link_libraries(driver 
                      pthread     # pthread or ${CMAKE_THREAD_LIBS_INIT}
                      ${catkin_LIBRARIES}
) 
