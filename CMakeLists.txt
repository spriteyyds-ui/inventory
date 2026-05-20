cmake_minimum_required(VERSION 3.8)
project(wheeltec_inventory_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

set(msg_files
  "msg/RecognizedNumber.msg"
  "msg/GapStatus.msg"
)

set(srv_files
  "srv/StartMission.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

include_directories(include)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(corridor_follower_node src/corridor_follower_node.cpp)
ament_target_dependencies(corridor_follower_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
)
target_link_libraries(corridor_follower_node ${cpp_typesupport_target})

add_executable(number_recognizer_node
  src/number_recognizer_node.cpp
  src/a4_detector.cpp
  src/digit_segmenter.cpp
  src/digit_classifier.cpp
)
ament_target_dependencies(number_recognizer_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  image_transport
  ament_index_cpp
)
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(number_recognizer_node
    ${OpenCV_LIBS}
    ${cpp_typesupport_target}
    yaml-cpp::yaml-cpp
  )
else()
  target_link_libraries(number_recognizer_node
    ${OpenCV_LIBS}
    ${cpp_typesupport_target}
    yaml-cpp
  )
endif()

add_executable(distance_estimator_node src/distance_estimator_node.cpp)
ament_target_dependencies(distance_estimator_node
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(distance_estimator_node ${cpp_typesupport_target})

add_executable(gap_detector_node src/gap_detector_node.cpp)
ament_target_dependencies(gap_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(gap_detector_node ${cpp_typesupport_target})

add_library(inventory_scan_modules STATIC
  src/scan_sequence_generator.cpp
  src/scan_sequence_executor.cpp
  src/inventory_scanner.cpp
  src/lift_controller.cpp
  src/web_api_client.cpp
)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(mission_manager_node
  rclcpp
  rclcpp_action
  sensor_msgs
  nav_msgs
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
  std_srvs
  ament_index_cpp
)
if(TARGET yaml-cpp::yaml-cpp)
  target_link_libraries(
    mission_manager_node
    ${cpp_typesupport_target}
    inventory_scan_modules
    yaml-cpp::yaml-cpp)
else()
  target_link_libraries(
    mission_manager_node
    ${cpp_typesupport_target}
    inventory_scan_modules
    yaml-cpp)
endif()

add_executable(offline_test
  test/offline_test.cpp
  src/a4_detector.cpp
  src/digit_segmenter.cpp
  src/digit_classifier.cpp
)
ament_target_dependencies(offline_test
  rclcpp
)
target_link_libraries(offline_test ${OpenCV_LIBS})

add_executable(test_scan_sequence_generator
  test/test_scan_sequence_generator.cpp
)
target_link_libraries(test_scan_sequence_generator inventory_scan_modules)

install(TARGETS
  corridor_follower_node
  number_recognizer_node
  distance_estimator_node
  gap_detector_node
  mission_manager_node
  offline_test
  test_scan_sequence_generator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/charge_status_gui.py
  scripts/inventory_auto_recharger.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models test docs
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
