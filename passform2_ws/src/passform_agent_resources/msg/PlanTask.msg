passform_agent_resources/SkillType type  # general skill information
string target_id # uuid of part to be affected (robot, part, effector, ..)
rcl_interfaces/Parameter[] required_parameter  # list of required parameter
rcl_interfaces/Parameter[] optional_parameter  # list of optional parameter
passform_agent_resources/Condition condition # geometric conditions for the task
string request_id # uuid of the request
