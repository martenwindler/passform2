uint8 STOP = 0
uint8 MOVE = 1
uint8 MOVE_TCP = 2
uint8 MOVE_BODY = 3

uint8 GRASP = 10
# uint8 ITEM_MOVE = 11
uint8 PRESSURE = 12
uint8 POSITION = 13
uint8 RELEASE = 14

uint8 TOOL_GET = 20
uint8 TOOL_USE = 21
uint8 TOOL_PUT = 22
uint8 STORE = 23
uint8 PROVIDE = 24

uint8 ASSEMBLY_PICKPLACE = 30
uint8 ASSEMBLY_SCREW = 31
uint8 ASSEMBLY_MOUNT = 32
uint8 ASSEMBLY_PLUG = 33

uint8 skill_type    # from list above
string name # human readable name of the skill
string description  # description of the skill
