cmake_minimum_required(VERSION 3.0.2)
project(localization_ndt)

add_compile_options(-std=c++14)

include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslib
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  ndt_omp
  tf2_ros          
  tf2
  localization_msgs
  diagnostic_msgs
  map_server
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgproc)

find_package(yaml-cpp REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    roslib
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    laser_geometry
    tf2_ros        
    tf2            
    ndt_omp
    localization_msgs
    diagnostic_msgs
    map_server
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_executable(ndt_localizer_node
  src/ndt_localizer_node.cpp
  src/initial_pose_manager.cpp
  src/local_relocalizer_2d.cpp
  src/relocalization_manager.cpp
  src/config/yaml_utils.cpp
  src/config/run_config.cpp
  src/config/params_yaml.cpp
  src/config/ndt_localizer_config.cpp
  src/ndt_localizer_node_init.cpp
  src/distance_field_2d.cpp
  src/dynamic_point_filter.cpp
  src/map_json_utils.cpp
)

target_link_libraries(ndt_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

add_executable(csm_localizer_node
  src/csm_localizer_node.cpp
  src/local_relocalizer_2d.cpp
  src/distance_field_2d.cpp
  src/dynamic_point_filter.cpp
)

add_executable(amcl_localizer_node
  src/amcl_localizer_node.cpp
  src/initial_pose_manager.cpp
  src/local_relocalizer_2d.cpp
  src/config/yaml_utils.cpp
  src/distance_field_2d.cpp
  src/dynamic_point_filter.cpp
  src/nav2_amcl_vendor/map/map.c
  src/nav2_amcl_vendor/map/map_cspace.cpp
  src/nav2_amcl_vendor/pf/pf.c
  src/nav2_amcl_vendor/pf/pf_kdtree.c
  src/nav2_amcl_vendor/pf/pf_pdf.c
  src/nav2_amcl_vendor/pf/pf_vector.c
  src/nav2_amcl_vendor/pf/eig3.c
  src/nav2_amcl_vendor/motion_model/differential_motion_model.cpp
  src/nav2_amcl_vendor/sensors/laser/laser.cpp
  src/nav2_amcl_vendor/sensors/laser/likelihood_field_model.cpp
  src/nav2_amcl_vendor/sensors/laser/likelihood_field_model_prob.cpp
)

add_dependencies(ndt_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(csm_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(amcl_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(csm_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

target_link_libraries(amcl_localizer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

if(HAVE_DRAND48)
  target_compile_definitions(amcl_localizer_node PRIVATE HAVE_DRAND48)
endif()
install(
  TARGETS 
  ndt_localizer_node
  csm_localizer_node
  amcl_localizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(
  DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(
  DIRECTORY map/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/map
)

install(
  DIRECTORY agv_urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/agv_urdf
)

install(
  FILES THIRD_PARTY_NOTICES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
