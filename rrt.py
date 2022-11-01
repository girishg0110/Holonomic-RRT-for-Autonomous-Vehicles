import numpy as np
from matplotlib import pyplot as plt

from tree import *
from collision import *
from sample import *


def rrt(robot, obstacles, start, goal, iter_n):
    n1 = 10
    n2 = 30
    dt = 0.1
    tree = Tree(robot, obstacles, start, goal)
    for it in range(iter_n):
        samplePoint = sample()
        nearest = tree.nearest(samplePoint)
        tree.extend(nearest, n1, n2, dt)
    nearest_goal = tree.nearest(goal)
    tree.add(nearest_goal, goal)

    path = []
    val = goal
    while (val != None):
        parent = tree.parent(val)
        path.append(val)
        val = parent

    return path[::-1]
