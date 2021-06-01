from typing import Tuple

import numpy as np


def get_motor_left_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    #res[100:150, 100:150] = 1
    #res[300:, 200:] = 1
    
    x = np.linspace([-2], 0, num=shape[1]//4).T
    y = np.linspace([2], 0, num=shape[0]//4)

    res[3*shape[0]//4:, shape[1]//8:3*shape[1]//8] = 1.0 / (2 * np.pi) * np.exp(-y**2 / 2) * np.exp(-x**2 / 2)

    return res


def get_motor_right_matrix(shape: Tuple[int, int]) -> np.ndarray:
    res = np.zeros(shape=shape, dtype="float32")  # write your function instead of this one
    #res[100:150, 100:300] = -1
    
    x = np.linspace([0], 2, num=shape[1]//4).T
    y = np.linspace([2], 0, num=shape[0]//2)
    res[shape[0]//2:, shape[1]//2:3*shape[1]//4] = 1.0 / (2 * np.pi) * np.exp(-y**2 / 2) * np.exp(-x**2 / 2)

    return res
