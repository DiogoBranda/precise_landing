cmake_minimum_required(VERSION 3.0.2)
project(precise_landing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  mavros
  mavros_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros

)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})


include(MavrosMavlink)



###################################
## catkin specific configuration ##
###################################

catkin_package(

)

###########
## Build ##
###########

include_directories(
${catkin_INCLUDE_DIRS}
${mavlink_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)


##add_executable(uavcontrol src/uavControler/uavcontrol.cpp)
add_executable(rosnode_preciselanding src/rosnode_preciselanding.cpp
    src/uavControler/uavcontrol.cpp src/camera/camera.cpp
    src/arucoCras/arucoCras.cpp src/sensorPose/sensorpose.cpp
    src/lidar/lidar.cpp)

add_executable(calib src/calib.cpp
    src/uavControler/uavcontrol.cpp src/camera/camera.cpp
    src/arucoCras/arucoCras.cpp src/sensorPose/sensorpose.cpp
    src/lidar/lidar.cpp)
##add_executable(uavcontroler src/UAVcontroler/uavcontroler.cpp src/UAVcontroler/uav.cpp)
##add_executable(termalDetect src/termaldetect/termalDetect.cpp src/termaldetect/camera.cpp src/termaldetect/myaruco.cpp src/termaldetect/arucopattern.cpp)

## src/objectdetecter/mypointcloud.cpp
##add_executable(markerdetecter src/markerdetecter/markerdetecter.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rosnode_preciselanding  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(calib  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##target_link_libraries(rosinterface_uav  ${catkin_LIBRARIES})
##target_link_libraries(objectdetecter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pcl_visualization)
##target_link_libraries(uavcontroler ${catkin_LIBRARIES})
##target_link_libraries(termalDetect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})





##target_link_libraries(markerdetecter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Teste.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
