import numpy as np
from matplotlib import pyplot as plt

class Robot:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.translation = [0,0]
        self.rotation = 0
    
    def set_pose(self, pose):
        x, y, theta = pose
        self.translation = [x,y]
        self.rotation = theta
    
    def transform(self):
        M = np.array([
            [np.cos(self.rotation), -np.sin(self.rotation), self.translation[0]],
            [np.sin(self.rotation), np.cos(self.rotation), self.translation[1]],
            [0, 0, 1]
        ])
        def get_vector(p):
            return np.array([p[0], p[1], 1])
        def unpackage(p):
            return [p[0], p[1]]

        points = [[-self.width/2, -self.height/2], [-self.width/2, self.height/2], [self.width/2, self.height/2], [self.width/2, -self.height/2]]
        return [unpackage(M @ get_vector(x)) for x in points]
        
    def kinematics(self, state, control):
        x, y, theta = state
        v, omega = control
        return [
            v * np.cos(theta + np.pi/2), v * np.sin(theta + np.pi/2), omega
        ]

    def propagate(self, state, controls, durations, dt):
        trajectory = [state]
        for control, duration in zip(controls, durations):
            for _ in range(duration):
                past = trajectory[-1]
                vels = self.kinematics(past, control)
                new_state = [
                    past[i] + vels[i]*dt for i in range(3)
                ]
                trajectory.append(new_state)
        return trajectory