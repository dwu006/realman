from math import pi
import numpy as np

class Door:
    def __init__(self):
        self.d = 0.4                # distance to travel after door seal
        self.w = 0.78               # width of the door (handle to hinge)
        self.theta_max = 4*pi/9
        self.x_o = np.zeros(3)
        self.x_i = None             # initial position of the base
        self.x_f = None             # final position of the base
