cmake_minimum_required(VERSION 2.8.3)
project(wifi_localization)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_filters
  sensor_msgs
  actionlib
)

find_package(Boost REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  WifiState.msg
  MaxWeight.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rpc_game
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

# include_directories("$ENV{EIGEN3_INCLUDE_DIR}")

# include_directories(include/gaussian_process)
include_directories(include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# include_directories(EIGEN3_INCLUDE_DIR)

install(PROGRAMS
  scripts/wifi_map_builder.py
  scripts/wifi_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare a C++ executable
add_executable(wifi_data_collector src/wifi_data_collector_node.cpp src/subscriber.cpp src/mapdata.cpp src/mapcollection.cpp)
add_executable(map_traverser src/map_traverser_node.cpp)
add_executable(wifi_localization src/wifi_localization_node.cpp src/gaussian_process/gaussian_process.cpp src/gaussian_process/kernel.cpp src/gaussian_process/optimizer.cpp)

add_dependencies(wifi_data_collector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(map_traverser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(wifi_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(wifi_data_collector
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

target_link_libraries(map_traverser
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )

target_link_libraries(wifi_localization
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        )