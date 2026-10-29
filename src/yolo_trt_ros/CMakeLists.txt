cmake_minimum_required(VERSION 3.0.2)
project(yolo_trt_ros)

find_package(catkin REQUIRED COMPONENTS rospy sensor_msgs std_msgs cv_bridge)
catkin_package(CATKIN_DEPENDS rospy sensor_msgs std_msgs cv_bridge)

catkin_install_python(PROGRAMS
  scripts/camera_publisher.py
  scripts/yolo_trt_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
