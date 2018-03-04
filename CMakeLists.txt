cmake_minimum_required(VERSION 2.8.3)
project(car107)
add_definitions(-std=c++11)

SET(EXECUTABLE_OUTPUT_PATH ".")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  image_geometry
  message_generation
)
find_package(OpenCV 3 REQUIRED)

add_message_files(
   FILES
   lineArray.msg
   crossPoint.msg
)

generate_messages(
    DEPENDENCIES
   geometry_msgs   nav_msgs   std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

find_package(Boost REQUIRED signals)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
#set(OpenCV_INCLUDE_DIRS "/usr/local/opencv3/include")
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS} 
)

#message(${OpenCV_LIBRARIES})
#message(${OpenCV_INCLUDE_DIRS})

add_executable(car107 ${PROJECT_SOURCE_DIR}/src/main.cpp 
                                    ${PROJECT_SOURCE_DIR}/src/utils.cpp
                                   ${PROJECT_SOURCE_DIR}/src/houghline.cpp
                                   ${PROJECT_SOURCE_DIR}/src/colorhistogram.cpp
                                   ${PROJECT_SOURCE_DIR}/src/histogram1d.cpp
                                   ${PROJECT_SOURCE_DIR}/src/imagecomparator.cpp
                                   ${PROJECT_SOURCE_DIR}/src/lbp.cpp
                                   ${PROJECT_SOURCE_DIR}/src/hsvcalchistogram.cpp
                                   ${PROJECT_SOURCE_DIR}/src/location.cpp
                                   )
target_link_libraries(car107 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})	

add_executable(readTest ${PROJECT_SOURCE_DIR}/src/readTest.cpp
                                    ${PROJECT_SOURCE_DIR}/src/utils.cpp
                                   ${PROJECT_SOURCE_DIR}/src/houghline.cpp
                                   ${PROJECT_SOURCE_DIR}/src/colorhistogram.cpp
                                   ${PROJECT_SOURCE_DIR}/src/histogram1d.cpp
                                   ${PROJECT_SOURCE_DIR}/src/imagecomparator.cpp
                                   ${PROJECT_SOURCE_DIR}/src/lbp.cpp
                                   ${PROJECT_SOURCE_DIR}/src/hsvcalchistogram.cpp
                                   ${PROJECT_SOURCE_DIR}/src/location.cpp
)
target_link_libraries(readTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(car_data ${PROJECT_SOURCE_DIR}/src/datasave.cpp
                                    ${PROJECT_SOURCE_DIR}/src/utils.cpp
                                   ${PROJECT_SOURCE_DIR}/src/houghline.cpp
                                   ${PROJECT_SOURCE_DIR}/src/colorhistogram.cpp
                                   ${PROJECT_SOURCE_DIR}/src/histogram1d.cpp
                                   ${PROJECT_SOURCE_DIR}/src/imagecomparator.cpp
                                   ${PROJECT_SOURCE_DIR}/src/lbp.cpp
                                   ${PROJECT_SOURCE_DIR}/src/hsvcalchistogram.cpp
)
target_link_libraries(car_data ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(laserMappingTest   ${PROJECT_SOURCE_DIR}/src/lasermappingtest.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/lasermapping.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/sensor.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/rangesensor.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/odometrysensor.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/rangereading
                                                        ${PROJECT_SOURCE_DIR}/src/sensorreading.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/smmap.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/nodeGridMap.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/pathcontrol.cpp
                                                        ${PROJECT_SOURCE_DIR}/src/findpath.cpp
)

target_link_libraries(laserMappingTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

add_executable(nodeMapTest ${PROJECT_SOURCE_DIR}/src/nodemaptest.cpp
                                                    ${PROJECT_SOURCE_DIR}/src/nodeGridMap.cpp
                                                    ${PROJECT_SOURCE_DIR}/src/findpath.cpp
)

target_link_libraries(nodeMapTest ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )

