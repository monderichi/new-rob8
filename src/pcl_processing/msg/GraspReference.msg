# Constants for grasp types
int8 GRASP_PALMAR = 0
int8 GRASP_LATERAL = 1
# int8 GRASP_PINCH = 2
# int8 GRASP_TRIPOD = 3
int8 GRASP_NONE = -1

std_msgs/Header header

int8 grasp_type
float32 wrist_orientation # Angle in radians or degrees (specify in implementation)
float32 grasp_size        # Opening size (e.g., distance between fingers/palm)