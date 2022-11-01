import numpy as np
from matplotlib import pyplot as plt

def sample():
    return [
        np.random.random() * 10,
        np.random.random() * 10,
        np.random.random() * 2 * np.pi - np.pi
    ]