cmake_minimum_required(VERSION 3.8)
project(passform_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_action
  ament_index_cpp
  behaviortree_cpp_v3
  behaviortree_ros2
  passform_msgs
  rcl_interfaces
  std_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(passform_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ament_index_cpp REQUIRED)

######################################################
# LINK
include_directories(include)

list(APPEND TARGET_LIST
  process_node
  # process_service
)

foreach(TARGET_NAME IN LISTS TARGET_LIST)
  add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
  target_include_directories(${TARGET_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${TARGET_NAME}
    ${THIS_PACKAGE_INCLUDE_DEPENDS}
  )
endforeach()


add_executable(test_forloop src/test_forloop.cpp src/nodes.cpp)
ament_target_dependencies(test_forloop ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
  test_forloop
  DESTINATION lib/${PROJECT_NAME})

######################################################
## TEST
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

######################################################
# INSTALL

install(TARGETS
  ${TARGET_LIST}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

######################################################

ament_export_include_directories(include)

ament_package()
