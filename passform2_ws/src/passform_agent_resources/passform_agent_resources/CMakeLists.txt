cmake_minimum_required(VERSION 3.5)
project(passform_msgs)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AreaOfInterest.msg"
  "msg/BaseStatus.msg"
  "msg/Bay.msg"
  "msg/BayStatus.msg"
  "msg/Blob.msg"
  "msg/Condition.msg"
  "msg/Cost.msg"
  "msg/Heartbeat.msg"
  "msg/Inventory.msg"
  "msg/Item.msg"
  "msg/Location.msg"
  "msg/Module.msg"
  "msg/ModuleStatus.msg"
  "msg/OperatorTask.msg"
  "msg/Skill.msg"
  "msg/SkillStatus.msg"
  "msg/SkillType.msg"
  "msg/Supply.msg"
  "msg/Task.msg"
  "msg/Tool.msg"
  "msg/Part.msg"
  "msg/Grasp.msg"
  "srv/CreateSkill.srv"
  "srv/Discover.srv"
  "srv/GetModules.srv"
  "srv/GetCost.srv"
  "srv/GetSkills.srv"
  "srv/PerformTask.srv"
  "srv/RegisterModule.srv"
  "srv/SetString.srv"
  "srv/UpdateInventory.srv"
  "action/Move.action"
  "action/Passform.action"
  "action/Sense.action"
  "action/Tool.action"
  "action/Wait.action"
  DEPENDENCIES std_msgs diagnostic_msgs geometry_msgs action_msgs rcl_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
