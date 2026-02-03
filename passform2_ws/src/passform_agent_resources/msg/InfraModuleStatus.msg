std_msgs/Header header
string uuid
diagnostic_msgs/DiagnosticStatus[] status
PlanSkill[] skills
WorldItem[] inventory  # list of uuid of items the module currently controls
WorldTool[] tools # list of tools the module currently controls
