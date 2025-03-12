cmake_minimum_required(VERSION 3.0.2)
project(xarm7)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 

install(FILES
  xarm7.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 