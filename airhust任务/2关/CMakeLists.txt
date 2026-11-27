cmake_minimum_required(VERSION 3.0.2)
project(airhust_tutorial26_circle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  nav_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp mavros_msgs nav_msgs tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# 可执行文件名 = launch 里的 type。若你仍把代码放在 src/template.cpp，
# 把这行改成 add_executable(flight_node_circle src/template.cpp) 即可。
add_executable(flight_node_circle src/flight_node_circle.cpp)
target_link_libraries(flight_node_circle ${catkin_LIBRARIES})
