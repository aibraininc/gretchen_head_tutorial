cmake_minimum_required(VERSION 2.8.3)
project(gretchen_head_controller)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  roscpp
  sensor_msgs
  trajectory_msgs
  control_msgs
)

catkin_package(
  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(head_hw_interface src/head_hw_interface.cpp)
add_dependencies(head_hw_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_hw_interface
   ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(head_manual_hw_interface src/head_manual_hw_interface.cpp)
add_dependencies(head_manual_hw_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(head_manual_hw_interface
   ${catkin_LIBRARIES}
)


