cmake_minimum_required(VERSION 3.0.2)
project(drone_bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  drone_backend
)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs drone_backend
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(flight_node src/flight_node.cpp)
add_dependencies(flight_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(flight_node ${catkin_LIBRARIES})
install(TARGETS flight_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
