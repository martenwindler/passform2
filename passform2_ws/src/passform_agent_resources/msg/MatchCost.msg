string request_id             # uuid of the request
string skill_uuid             # uuid of the skill this response is valid for
string action_topic           # topic to start the action
builtin_interfaces/Duration duration    # total time needed to perform the skill
builtin_interfaces/Time earliest_start  # epoch of earliest possible time to start skill
float32 energy                # consumed energy in Wh
