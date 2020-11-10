from enum import Enum


class ConstraintType(Enum):
    Place = 0
    Frame = 1
    Vacuum = 2
    Grasp2 = 3

OPPOSITE_DICT={
    "top": "bottom",
    "bottom": "top",
    "right": "left",
    "left": "right",
    "front": "back",
    "back": "front"
}