cmake_minimum_required(VERSION 3.8)
project(passform_agent_planning)

# mehr Warnungen auf GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1) ament-CMake und Abhängigkeiten finden
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)



# 3) Deine Python-Skripte installieren
#    Wir installieren sie nach lib/<pkg>/module_node und path_requestor
install(PROGRAMS
  scripts/agent_node.py
  scripts/monitor_node.py
  scripts/central_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# 4) Launch- und Config-Verzeichnisse mitinstallieren
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 5) Rosidl-Abhängigkeit exportieren
ament_export_dependencies(rosidl_default_runtime)

ament_package()
