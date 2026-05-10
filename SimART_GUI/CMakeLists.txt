cmake_minimum_required(VERSION 3.10)
project(airsim_gui_UErealtime)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOMOC ON)

option(AIRSIM_GUI_ENABLE_AIRSIM "Enable AirSim mesh API support and C++ RPC live view" OFF)
set(AIRSIM_CLIENT_ROOT "" CACHE PATH "Path to an AirSim checkout when enabling AirSim mesh import and C++ RPC live view")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  rf_msgs
  sionna_sys_msgs
  sionna_beam_msgs
)

find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL Gui Quick QuickWidgets Qml Network Xml)
find_package(Threads REQUIRED)
find_package(VTK REQUIRED)
find_package(assimp REQUIRED)
include(${VTK_USE_FILE})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs std_msgs sensor_msgs rf_msgs sionna_sys_msgs sionna_beam_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
)

set(AIRSIM_EXTRA_SOURCES)
set(AIRSIM_EXTRA_LIBS)

if(AIRSIM_GUI_ENABLE_AIRSIM)
  if(NOT AIRSIM_CLIENT_ROOT AND DEFINED ENV{AIRSIM_CLIENT_ROOT})
    set(AIRSIM_CLIENT_ROOT "$ENV{AIRSIM_CLIENT_ROOT}" CACHE PATH "Path to an AirSim checkout" FORCE)
  endif()

  set(_AIRSIM_CLIENT_ROOT_CANDIDATES
    "${CMAKE_CURRENT_SOURCE_DIR}/../AirSim"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../AirSim"
    "$ENV{HOME}/AirSim"
  )
  if(NOT AIRSIM_CLIENT_ROOT)
    foreach(_candidate IN LISTS _AIRSIM_CLIENT_ROOT_CANDIDATES)
      if(_candidate AND EXISTS "${_candidate}/AirLib/include")
        set(AIRSIM_CLIENT_ROOT "${_candidate}" CACHE PATH "Path to an AirSim checkout" FORCE)
        break()
      endif()
    endforeach()
  endif()
  if(NOT AIRSIM_CLIENT_ROOT)
    message(FATAL_ERROR "AIRSIM_GUI_ENABLE_AIRSIM=ON requires AIRSIM_CLIENT_ROOT to be set with -DAIRSIM_CLIENT_ROOT=/path/to/AirSim, or the AIRSIM_CLIENT_ROOT environment variable to point to an AirSim checkout.")
  endif()

  get_filename_component(AIRSIM_CLIENT_ROOT "${AIRSIM_CLIENT_ROOT}" ABSOLUTE)
  unset(AIRSIM_RPC_LIB CACHE)

  set(AIRSIM_AIRLIB_INCLUDE "${AIRSIM_CLIENT_ROOT}/AirLib/include")
  set(AIRSIM_RPCLIB_INCLUDE "${AIRSIM_CLIENT_ROOT}/AirLib/deps/rpclib/include")
  set(AIRSIM_MAVLINK_INCLUDE "${AIRSIM_CLIENT_ROOT}/AirLib/deps/MavLinkCom/include")
  if(NOT EXISTS "${AIRSIM_AIRLIB_INCLUDE}")
    message(FATAL_ERROR "AirLib headers not found under ${AIRSIM_AIRLIB_INCLUDE}")
  endif()

  include_directories(
    ${AIRSIM_AIRLIB_INCLUDE}
    ${AIRSIM_RPCLIB_INCLUDE}
    ${AIRSIM_MAVLINK_INCLUDE}
  )
  add_definitions(-DAIRSIM_GUI_ENABLE_AIRSIM)

  foreach(_src
    "${AIRSIM_CLIENT_ROOT}/AirLib/src/api/RpcLibClientBase.cpp"
    "${AIRSIM_CLIENT_ROOT}/AirLib/src/vehicles/multirotor/api/MultirotorRpcLibClient.cpp"
  )
    if(EXISTS "${_src}")
      list(APPEND AIRSIM_EXTRA_SOURCES "${_src}")
    else()
      message(FATAL_ERROR "Required AirSim client source not found: ${_src}")
    endif()
  endforeach()

  find_library(AIRSIM_RPC_LIB
    NAMES rpc rpclib
    PATHS
      "${AIRSIM_CLIENT_ROOT}/AirLib/deps/rpclib/lib"
      "${AIRSIM_CLIENT_ROOT}/AirLib/deps/rpclib/build"
      "${AIRSIM_CLIENT_ROOT}/AirLib/deps/rpclib/build/lib"
      "${AIRSIM_CLIENT_ROOT}/build_debug/output/lib"
      "${AIRSIM_CLIENT_ROOT}/build_release/output/lib"
      "${AIRSIM_CLIENT_ROOT}/cmake/output/lib"
    NO_DEFAULT_PATH)
  if(AIRSIM_RPC_LIB)
    list(APPEND AIRSIM_EXTRA_LIBS ${AIRSIM_RPC_LIB})
    message(STATUS "Using rpclib: ${AIRSIM_RPC_LIB}")
  else()
    message(FATAL_ERROR "Could not find rpclib for AirSim. Build AirSim dependencies first, then set AIRSIM_CLIENT_ROOT to that checkout.")
  endif()
endif()

set(HEADERS
  include/airsim_gui_UErealtime/MainWindow.h
  include/airsim_gui_UErealtime/Scene3DWidget.h
  include/airsim_gui_UErealtime/RosBridge.h
  include/airsim_gui_UErealtime/DemoController.h
  include/airsim_gui_UErealtime/JsonConfig.h
  include/airsim_gui_UErealtime/AppTypes.h
  include/airsim_gui_UErealtime/TransformUtils.h
  include/airsim_gui_UErealtime/FrameTransformManager.h
  include/airsim_gui_UErealtime/SceneTypes.h
  include/airsim_gui_UErealtime/ISceneProvider.h
  include/airsim_gui_UErealtime/FileSceneProvider.h
  include/airsim_gui_UErealtime/AirSimSceneProvider.h
  include/airsim_gui_UErealtime/OsmSceneProvider.h
  include/airsim_gui_UErealtime/OsmMapWidget.h
  include/airsim_gui_UErealtime/OsmImportDialog.h
  include/airsim_gui_UErealtime/SceneManager.h
  include/airsim_gui_UErealtime/AirSimViewController.h
  include/airsim_gui_UErealtime/UeViewWidget.h
  include/airsim_gui_UErealtime/SionnaPreviewWidget.h
  include/airsim_gui_UErealtime/CkmMapWidget.h
)

set(SOURCES
  src/main.cpp
  src/MainWindow.cpp
  src/Scene3DWidget.cpp
  src/RosBridge.cpp
  src/JsonConfig.cpp
  src/TransformUtils.cpp
  src/FrameTransformManager.cpp
  src/DemoController.cpp
  src/FileSceneProvider.cpp
  src/AirSimSceneProvider.cpp
  src/OsmSceneProvider.cpp
  src/OsmMapWidget.cpp
  src/OsmImportDialog.cpp
  src/SceneManager.cpp
  src/AirSimViewController.cpp
  src/UeViewWidget.cpp
  src/SionnaPreviewWidget.cpp
  src/CkmMapWidget.cpp
  ${AIRSIM_EXTRA_SOURCES}
)

add_executable(${PROJECT_NAME}
  ${HEADERS}
  ${SOURCES}
  resources.qrc
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
  Qt5::OpenGL
  Qt5::Gui
  Qt5::Quick
  Qt5::QuickWidgets
  Qt5::Qml
  Qt5::Network
  Qt5::Xml
  ${catkin_LIBRARIES}
  ${VTK_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  ${AIRSIM_EXTRA_LIBS}
  Threads::Threads
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

catkin_install_python(PROGRAMS
  scripts/sionna_sim_only_topic2.py
  scripts/reprocess_rosbag_with_sionna.py
  scripts/path_to_rviz_odom.py
  scripts/sionna_internal_socket_bridge.py
  scripts/sionna_beam_topic_utils.py
  scripts/sionna_tf_frame_utils.py
  scripts/export_sionna_codebook.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  qml/UeLiveView.qml
  qml/UeStationCameraView.qml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/qml
)

install(PROGRAMS
  scripts/airsim_image_bridge.py
  scripts/sionna_embed_launcher.py
  scripts/sionna_resim_socket_worker.py
  scripts/sionna_ckm_generator.py
  scripts/scene_bounds_from_xml.py
  scripts/sionna_beam_topic_utils.py
  scripts/sionna_tf_frame_utils.py
  scripts/export_sionna_codebook.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY third_party/sionna_rt_gui/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/third_party/sionna_rt_gui
)
