import numpy as np
from matplotlib import pyplot as plt

from robot import *

def edgeVectors(vertices):
    edges = []
    for i, currVertex in enumerate(vertices):
        nextVertex = vertices[(i + 1) % len(vertices)]
        edges.append(
            [nextVertex[0] - currVertex[0], nextVertex[1] - currVertex[1]]
        )
    return edges

def isCollisionFree(robot, point, obstacles):
    robot.set_pose(point)
    newRobot = robot.transform()
    # 1. Boundaries
    maxY = max([r[1] for r in newRobot])
    minY = min([r[1] for r in newRobot])
    maxX = max([r[0] for r in newRobot])
    minX = min([r[0] for r in newRobot])
    if (minX < 0) or (maxX > 10) or (minY < 0) or (maxY > 10):
        return False

    def unit_normal(vector):
        x, y = vector
        return [y/(1e-6 + (x**2 + y**2)**0.5), -x/(1e-6 + (x**2 + y**2)**0.5)]

    def dot(vector1, vector2):
        return vector1[0] * vector2[0] + vector1[1] * vector2[1]

    # 2. Obstacles
    for obs in obstacles:
        obsCollided = True
        edges = edgeVectors(newRobot) + edgeVectors(obs)
        edgeNormals = [unit_normal(edge) for edge in edges]
        for normal in edgeNormals:
            robotProjection = [dot(r, normal) for r in newRobot]
            obsProjection = [dot(o, normal) for o in obs]
            if (max(robotProjection) < min(obsProjection)) or (max(obsProjection) < min(robotProjection)):
                obsCollided = False
                break
        if (obsCollided):
            return False

    return True
