# Constants for shape types
int8 SHAPE_SPHERE = 0
int8 SHAPE_CYLINDER = 1
int8 SHAPE_BOX = 2
int8 SHAPE_UNKNOWN = -1

std_msgs/Header header

int8 shape_type

# Center point of the primitive
geometry_msgs/Point center

# Orientation vectors
# Sphere: Empty
# Cylinder: 1 vector (axis direction)
# Box: 3 vectors (principal axes)
geometry_msgs/Vector3[] orientation

# Dimensions
# Sphere: 1 value (radius)
# Cylinder: 2 values (radius, length)
# Box: 3 values (size along each principal axis)
float32[] dimensions