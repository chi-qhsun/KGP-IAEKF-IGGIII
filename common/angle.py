import math
import numpy as np

class Angle:

    D2R = (math.pi / 180.0)
    R2D = (180.0 / math.pi)

    # radian - degree conversion
    @staticmethod
    def rad2deg(rad: float):
        return rad * Angle.R2D

    @staticmethod
    def deg2rad(deg: float):
        return deg * Angle.D2R

    # matrix
    @staticmethod
    def rad2deg(arr: np.ndarray):
        return arr * Angle.R2D

    @staticmethod
    def deg2rad(arr: np.ndarray):
        return arr * Angle.D2R

