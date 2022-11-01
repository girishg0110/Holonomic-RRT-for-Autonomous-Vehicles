import numpy as np
from matplotlib import pyplot as plt
from collision import *
from robot import *

class TreeNode:
    def __init__(self, value):
        self.value = value
        self.children = []

    def print_treenode(self, depth=0):
        print(depth * "---->", self.value, sep='')
        for c in self.children:
            c.print_treenode(depth + 1)


class Tree:
    def __init__(self, robot, obstacles, start, goal):
        self.robot = robot
        self.obstacles = obstacles
        self.goal = goal
        self.root = TreeNode(start)

    def add(self, point1, point2):
        nodes = [self.root]
        while(len(nodes) != 0):
            curr = nodes.pop(0)
            if (curr.value == point1):
                curr.children.append(TreeNode(point2))
                return True
            else:
                nodes.extend(curr.children)
        return False

    def exists(self, point):
        nodes = [self.root]
        while(len(nodes) != 0):
            curr = nodes.pop(0)
            if (curr.value == point):
                return True
            else:
                nodes.extend(curr.children)
        return False

    def parent(self, point):
        nodes = [self.root]
        while(len(nodes) != 0):
            curr = nodes.pop(0)
            for node in curr.children:
                if point == node.value:
                    return curr.value
            else:
                nodes.extend(curr.children)
        return None

    def angle_dist(theta1, theta2):
        theta1 %= (2 * np.pi)
        theta2 %= (2 * np.pi)
        diff = theta2 - theta1
        if (-np.pi <= diff <= np.pi):
            return diff
        else:
            return diff - 2*np.pi if diff > 0 else 2*np.pi + diff

    def sq_distance(point1, point2):
        if (point1 == None) or (point2 == None):
            return 0
        return (point1[0]-point2[0])**2 + (point1[1]-point2[1])**2 + Tree.angle_dist(point1[2], point2[2])**2

    def nearest(self, point):
        closestNode = None
        closestDistance = np.inf
        nodes = [self.root]
        while(len(nodes) != 0):
            curr = nodes.pop(0)
            currDistance = Tree.sq_distance(curr.value, point)
            if closestDistance > currDistance:
                closestDistance = currDistance
                closestNode = curr.value
            nodes.extend(curr.children)
        return closestNode

    def extend(self, point, n1, n2, dt):
        def get_u():
            return [
                np.random.random() * 1, 
                np.random.random() * (2 * np.pi) - np.pi
            ]
        def get_n():
            return np.random.randint(n1, n2)
            
        u = get_u()
        n = get_n()
        trajectory = self.robot.propagate(point, [u], [n], dt)
        for i in range(1, len(trajectory)):
            if not isCollisionFree(self.robot, trajectory[i], self.obstacles):
                self.add(point, trajectory[i-1])
                return trajectory[i-1]

        self.add(point, trajectory[-1])
        return trajectory[-1]

    def print_tree(self):
        self.root.print_treenode()

    def plot_tree(self):
        def draw_polygon(poly, color, edge=True):
            def extract_index(arr, index):
                return [a[index] for a in arr]
            plt.fill(extract_index(poly, 0), extract_index(poly, 1),
                     color=color, edgecolor="black" if edge else "white")

        plt.figure(figsize=(10, 10))
        plt.axis((0, 10, 0, 10))
        for obs in self.obstacles:
            draw_polygon(obs, "grey")

        self.robot.set_pose(self.root.value)
        startCoords = self.robot.transform()
        draw_polygon(startCoords, "red")
        
        self.robot.set_pose(self.goal)
        goalCoords = self.robot.transform()
        draw_polygon(goalCoords, "blue")

        nodes = [self.root]
        while (len(nodes) > 0):
            curr = nodes.pop()
            for c in curr.children:
                plt.plot([curr.value[0], c.value[0]], [
                         curr.value[1], c.value[1]], color="black")
            nodes.extend(curr.children)
            
        plt.show()