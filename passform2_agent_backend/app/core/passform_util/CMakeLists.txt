cmake_minimum_required(VERSION 3.5)
project(passform_util)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

include_directories(include/${PROJECT_NAME})
set(HEADER_FILES include/${PROJECT_NAME}/helper.hpp)
add_library(helper src/helper.cpp ${HEADER_FILES} )
# ament_target_dependencies(module passform_msgs)
ament_export_targets(helper HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS helper
  EXPORT helper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include #/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
#  scripts/py_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
