cmake_minimum_required(VERSION 3.0.2)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  serial
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs serial
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/imu_driver.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
