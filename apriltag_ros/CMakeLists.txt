cmake_minimum_required(VERSION 3.8)
project(apriltag_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the build type. Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# We default to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Coverage" "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "srv/AnalyzeSingleImage.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(include)

##################
## Common library #
##################

add_library(${PROJECT_NAME}_common SHARED src/common_functions.cpp)
add_dependencies(${PROJECT_NAME}_common ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_common
  "${cpp_typesupport_target}"
  apriltag::apriltag
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME}_common
  rclcpp
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

#########################
## Continuous detector  #
#########################

add_library(${PROJECT_NAME}_continuous_detector SHARED src/continuous_detector.cpp)
target_link_libraries(${PROJECT_NAME}_continuous_detector ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_continuous_detector
  rclcpp
  rclcpp_components
  image_transport
  sensor_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_continuous_detector
  PLUGIN "apriltag_ros::ContinuousDetector"
  EXECUTABLE ${PROJECT_NAME}_continuous_node
)

############################
## Single image detector   #
############################

add_library(${PROJECT_NAME}_single_image_detector SHARED src/single_image_detector.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_detector ${PROJECT_NAME}_common)
ament_target_dependencies(${PROJECT_NAME}_single_image_detector
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(${PROJECT_NAME}_single_image_detector
  PLUGIN "apriltag_ros::SingleImageDetector"
  EXECUTABLE ${PROJECT_NAME}_single_image_server_node
)

add_executable(${PROJECT_NAME}_single_image_client_node src/apriltag_ros_single_image_client_node.cpp)
target_link_libraries(${PROJECT_NAME}_single_image_client_node "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_single_image_client_node rclcpp sensor_msgs)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_common
  ${PROJECT_NAME}_continuous_detector
  ${PROJECT_NAME}_single_image_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Note: the *_continuous_node and *_single_image_server_node executables are
# generated and installed automatically by rclcpp_components_register_node()
# above; only the plain client node needs an explicit install here.
install(TARGETS
  ${PROJECT_NAME}_single_image_client_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/analyze_image DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_common ${PROJECT_NAME}_continuous_detector ${PROJECT_NAME}_single_image_detector)
ament_export_dependencies(rclcpp rclcpp_components cv_bridge geometry_msgs image_geometry image_transport sensor_msgs std_msgs tf2 tf2_ros rosidl_default_runtime)

ament_package()
