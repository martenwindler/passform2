cmake_minimum_required(VERSION 3.5)
project(passform_agent_resources)

# C++ Standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 1. Abhängigkeiten
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# 2. Interfaces generieren
rosidl_generate_interfaces(${PROJECT_NAME}
  # --- MESSAGES ---
  "msg/AgentAnnounce.msg"
  "msg/AgentInfo.msg"
  "msg/AreaOfInterest.msg"
  "msg/Award.msg"
  "msg/BaseStatus.msg"
  "msg/Bay.msg"
  "msg/BayStatus.msg"
  "msg/Blob.msg"
  "msg/CFP.msg"
  "msg/Condition.msg"
  "msg/Cost.msg"
  "msg/CostPropagation.msg"
  "msg/Grasp.msg"
  "msg/GridPosition.msg"
  "msg/Heartbeat.msg"
  "msg/Inventory.msg"
  "msg/Item.msg"
  "msg/Location.msg"
  "msg/Module.msg"
  "msg/ModuleStatus.msg"
  "msg/Motion.msg"
  "msg/OperatorTask.msg"
  "msg/Part.msg"
  "msg/PathComplete.msg"
  "msg/PathRequest.msg"
  "msg/PathResult.msg"
  "msg/Proposal.msg"
  "msg/Skill.msg"
  "msg/SkillStatus.msg"
  "msg/SkillType.msg"
  "msg/Supply.msg"
  "msg/Task.msg"
  "msg/Tool.msg"
  
  # --- SERVICES ---
  "srv/CreateSkill.srv"
  "srv/Discover.srv"
  "srv/GetCost.srv"
  "srv/GetModules.srv"
  "srv/GetSkills.srv"
  "srv/PerformTask.srv"
  "srv/RegisterModule.srv"
  "srv/SetString.srv"
  "srv/UpdateInventory.srv"
  
  # --- ACTIONS ---
  "action/Move.action"
  "action/Passform.action"
  "action/Sense.action"
  "action/Tool.action"
  "action/Wait.action"
  
  DEPENDENCIES std_msgs diagnostic_msgs geometry_msgs action_msgs rcl_interfaces
)

# 3. Installation der Ressourcen
install(DIRECTORY config meshes skills urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()