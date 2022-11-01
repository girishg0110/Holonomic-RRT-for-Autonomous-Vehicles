import numpy as np
from matplotlib import pyplot as plt
from robot import *

def draw_polygon(poly, color, edge=True):
    def extract_index(arr, index):
        return [a[index] for a in arr]
    plt.fill(extract_index(poly, 0), extract_index(poly, 1),
             color=color, edgecolor="black" if edge else "white")

def visualize_problem(robot, obstacles, start, goal):
    plt.figure(figsize=(10, 10))
    plt.axis((0, 10, 0, 10))
    for obs in obstacles:
        draw_polygon(obs, "grey")

    robot.set_pose(start)
    startCoords = robot.transform()
    draw_polygon(startCoords, "red")

    robot.set_pose(goal)
    goalCoords = robot.transform()
    draw_polygon(goalCoords, "blue")

    robot.set_pose([0,0,0])
    plt.show()

def visualize_points(points, robot, obstacles, start, goal):
    plt.figure(figsize=(10, 10))
    plt.axis((0, 10, 0, 10))
    for obs in obstacles:
        draw_polygon(obs, "grey")

    for point in points:
        robot.set_pose(point)
        pointCoords = robot.transform()
        draw_polygon(pointCoords, "orange")

    robot.set_pose(start)
    startCoords = robot.transform()
    draw_polygon(startCoords, "red")

    robot.set_pose(goal)
    goalCoords = robot.transform()
    draw_polygon(goalCoords, "blue")

    robot.set_pose([0,0,0])
    plt.show()

def visualize_path(robot, obstacles, path):
    plt.figure(figsize=(10, 10))
    plt.axis((0, 10, 0, 10))
    for obs in obstacles:
        draw_polygon(obs, "grey")

    robot.set_pose(path[0])
    goalCoords = robot.transform()
    draw_polygon(goalCoords, "green")

    for i in range(len(path)-1):
        prev = path[i]
        p = path[i+1]
        robot.set_pose(p)
        plt.plot([prev[0], p[0]], [prev[1], p[1]], color="black")
        draw_polygon(robot.transform(), "green")
        
    plt.show()

def visualize_trajectory(robot, obstacles, start, goal, trajectory):
    plt.figure(figsize=(10, 10))
    plt.axis((0, 10, 0, 10))
    for obs in obstacles:
        draw_polygon(obs, "grey")

    robot.set_pose(start)
    startCoords = robot.transform()
    draw_polygon(startCoords, "red")

    for i in range(1, len(trajectory)):
        prev = trajectory[i-1]
        robot.set_pose(prev)
        prevCoords = robot.transform()

        state = trajectory[i]
        robot.set_pose(state)
        stateCoords = robot.transform()
        draw_polygon(stateCoords, "green")

        plt.plot([prevCoords[0][0], stateCoords[0][0]], [prevCoords[0][1], stateCoords[0][1]], color = "black")

    robot.set_pose(goal)
    goalCoords = robot.transform()
    draw_polygon(goalCoords, "blue")

    plt.show()


