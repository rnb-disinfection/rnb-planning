from enum import Enum

##
# @class GEOTYPE
# @brief Geometry type enumeration
class GEOTYPE(Enum):
    SPHERE = 0
    CAPSULE = 1
    BOX = 2
    MESH = 3
    ARROW = 4
    CYLINDER = 5
    PLANE = 6