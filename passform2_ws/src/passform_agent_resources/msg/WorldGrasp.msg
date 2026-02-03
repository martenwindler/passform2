uint8 FINGER=0
uint8 AREA=1

uint8 FORCE=0
uint8 FIT=1
uint8 VACUUM=2
uint8 ELECTROSTATIC=3

int32 geometry_type  # FINGER, AREA
int32 effect_type # FORCE, FIT, ..
geometry_msgs/Pose[] points # list of grasping points to be grasped
float32 max_force # maximum grasping force in N
