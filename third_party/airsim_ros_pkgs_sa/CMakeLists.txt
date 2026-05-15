cmake_minimum_required(VERSION 3.10.0)
project(airsim_ros_pkgs_sa)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

# add_definitions(-DAIRSIM_ROS_WRAPPER_DEBUG)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wstrict-null-sentinel -Wno-unused")

# ---------------------------------------------------------------------------
# Performance flags
# ---------------------------------------------------------------------------
# NOTE: This package polls high-rate sensors and publishes images; building without
# optimizer can cap publish rates. Keep Debug-friendly flags in Debug builds.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O0 -g)
else()
  add_compile_options(-O3 -DNDEBUG)
endif()

# Optional: tune for the build machine (comment out if you need portable binaries)
add_compile_options(-march=native)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10")

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  message_generation
  nav_msgs
  # nodelet
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geographic_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

add_message_files(
  FILES
  GimbalAngleEulerCmd.msg
  GimbalAngleQuatCmd.msg
  GPSYaw.msg
  VelCmd.msg
  VelCmdGroup.msg
  CarControls.msg
  CarState.msg
  Altimeter.msg
  Environment.msg
)

add_service_files(
  FILES
  SetGPSPosition.srv
  Takeoff.srv
  TakeoffGroup.srv
  Land.srv
  LandGroup.srv
  Reset.srv
  SetLocalPosition.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  geographic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS message_runtime roscpp std_msgs # nodelet 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(airsim_settings_parser_ex src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser_ex ${catkin_LIBRARIES} AirLib)

add_library(pd_position_controller_simple_ex src/pd_position_controller_simple.cpp)
add_dependencies(pd_position_controller_simple_ex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pd_position_controller_simple_ex ${catkin_EXPORTED_TARGETS})

add_library(airsim_ros_ex src/airsim_ros_wrapper.cpp)
add_dependencies(airsim_ros_ex ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_ros_ex ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib airsim_settings_parser_ex)

add_executable(airsim_node_ex src/airsim_node.cpp)
target_link_libraries(airsim_node_ex airsim_ros_ex ${catkin_LIBRARIES} AirLib)

add_executable(pd_position_controller_simple_node_ex src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node_ex pd_position_controller_simple_ex airsim_ros_ex ${catkin_LIBRARIES} AirLib)

install(TARGETS
    #list of nodes
    airsim_node_ex
    pd_position_controller_simple_node_ex
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros_ex
    pd_position_controller_simple_ex
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  # settings.json
  # nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# for shipping binaries. run `$ catin_make install`
install(DIRECTORY
  launch
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # PATTERN ".git" EXCLUDE
)
