import numpy as np
from matplotlib import pyplot as plt
from robot import *

def parse_problem(world_file, problem_file):
    robot = None
    obstacles = []
    with open(world_file, 'r') as world:
        for i, line in enumerate(world.readlines()):
            line_inp = [float(x) for x in line.split(" ")]
            if (i == 0):
                width, height = line_inp
                robot = Robot(width, height)
            else: 
                obstacles.append(
                    [line_inp[j:j+2] for j in range(0, len(line_inp), 2)]
                )

    problem_set = []
    with open(problem_file, 'r') as problems:
        for i, line in enumerate(problems.readlines()):
            line_inp = [float(x) for x in line.split(" ")]
            problem_set.append(
                [line_inp[0:3], line_inp[3:6]]
            )
    
    return (robot, obstacles, problem_set)
