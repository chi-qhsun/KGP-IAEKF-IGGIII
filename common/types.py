import numpy as np


class GNSS:
    def __init__(self, time: float, blh: np.ndarray, std: np.ndarray, isvalid: bool):
        self.time = time

        self.blh = blh
        self.std = std

        self.isvalid = isvalid

    def __init__(self):
        self.time = None

        self.blh = None
        self.std = None

        self.isvalid = None


class IMU:
    def __init__(self, time: float, dt: float, dtheta: np.ndarray, dvel: np.ndarray, odovel: float):
        self.time = time
        self.dt = dt

        self.dtheta = dtheta
        self.dvel = dvel

        self.odovel = odovel

    def __init__(self):
        self.time = 0
        self.dt = 0.0

        self.dtheta = 0
        self.dvel = 0

        self.odovel = None


class Pose:
    def __init__(self, R: np.ndarray, t: np.ndarray):
        self.R = R
        self.t = t
