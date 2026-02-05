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
  # --- BASIS-TYPEN ZUERST (Wichtig für Jazzy!) ---
  "msg/GridPosition.msg"      # Wird von AgentInfo/Announce genutzt
  "msg/AgentInfo.msg"         # Wird von AgentAnnounce/PathResult genutzt
  "msg/AgentState.msg"
  "msg/AgentHeartbeat.msg"
  
  # --- RESTLICHE NACHRICHTEN ---
  "msg/AgentAnnounce.msg"
  "msg/InfraBaseStatus.msg"
  "msg/InfraBay.msg"
  "msg/InfraBayStatus.msg"
  "msg/InfraModule.msg"
  "msg/InfraModuleStatus.msg"
  "msg/InfraSupply.msg"
  "msg/MatchAward.msg"
  "msg/MatchCFP.msg"
  "msg/MatchCost.msg"
  "msg/MatchProposal.msg"
  "msg/NavMotion.msg"
  "msg/NavPathComplete.msg"
  "msg/NavPathRequest.msg"
  "msg/NavPathResult.msg"
  "msg/PlanCondition.msg"
  "msg/PlanOperatorTask.msg"
  "msg/PlanSkill.msg"
  "msg/PlanSkillStatus.msg"
  "msg/PlanSkillType.msg"
  "msg/PlanTask.msg"
  "msg/PlanTaskCost.msg"
  "msg/RangerActuatorState.msg"
  "msg/RangerActuatorStateArray.msg"
  "msg/RangerDriverState.msg"
  "msg/RangerMotionState.msg"
  "msg/RangerMotorState.msg"
  "msg/RangerSystemState.msg"
  "msg/WorldAreaOfInterest.msg"
  "msg/WorldBlob.msg"
  "msg/WorldGrasp.msg"
  "msg/WorldInventory.msg"
  "msg/WorldItem.msg"
  "msg/WorldLocation.msg"
  "msg/WorldPart.msg"
  "msg/WorldTool.msg"
  
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
  "action/Discovery.action"
  "action/Move.action"
  "action/MoveBody.action"
  "action/MoveTcp.action"
  "action/Sense.action"
  "action/System.action"
  "action/Tool.action"
  "action/Wait.action"
  
  DEPENDENCIES std_msgs diagnostic_msgs geometry_msgs action_msgs rcl_interfaces
)

# 3. Installation der Ressourcen für das Backend
install(DIRECTORY config meshes skills specifications urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()