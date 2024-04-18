from common.angle import *
import numpy as np
from pyquaternion import Quaternion

class Attitude:
    def __init__(self, qbn: Quaternion = Quaternion(0,0,0,0), cbn: np.ndarray = np.zeros((3,3)), euler: np.ndarray = np.zeros((3,))):
        self.qbn = qbn
        self.cbn = cbn
        self.euler = euler

class PVA:
    def __init__(self, pos: np.ndarray = np.zeros((3,)), vel: np.ndarray = np.zeros((3,)), att: Attitude = Attitude()):
        self.pos = pos
        self.vel = vel
        self.att = att

    # def __init__(self):
    #     self.pos = None
    #     self.vel = None
    #     self.att = None

class ImuError:
    def __init__(self, gyrbias: np.ndarray = np.zeros((3,)), accbias: np.ndarray = np.zeros((3,)), gyrscale: np.ndarray = np.zeros((3,)), accscale: np.ndarray = np.zeros((3,))):
        self.gyrbias = gyrbias
        self.accbias = accbias
        self.gyrscale = gyrscale
        self.accscale = accscale

    # def __init__(self):
    #     self.gyrbias = None
    #     self.accbias = None
    #     self.gyrscale = None
    #     self.accscale = None

class NavState:
    def __init__(self, pos: np.ndarray = np.zeros((3,)), vel: np.ndarray = np.zeros((3,)), euler: np.ndarray = np.zeros((3,)), imuerror: ImuError = ImuError()):
        self.pos = pos
        self.vel = vel
        self.euler = euler
        self.imuerror = imuerror

    # def __init__(self):
    #     self.pos = None
    #     self.vel = None
    #     self.euler = None
    #     self.imuerror = None

class ImuNoise:
    def __init__(self, gyr_arw: np.ndarray = np.zeros((3,)), acc_vrw: np.ndarray = np.zeros((3,)), gyrbias_std: np.ndarray = np.zeros((3,)),
                 accbias_std: np.ndarray = np.zeros((3,)), gyrscale_std: np.ndarray = np.zeros((3,)), accscale_std: np.ndarray = np.zeros((3,)), corr_time: float = 0.0):
        self.gyr_arw = gyr_arw
        self.acc_vrw = acc_vrw
        self.gyrbias_std = gyrbias_std
        self.accbias_std = accbias_std
        self.gyrscale_std = gyrscale_std
        self.accscale_std = accscale_std
        self.corr_time = corr_time

class GINSOptions:
    def __init__(self, initstate: NavState = NavState(), initstate_std: NavState = NavState(), imunoise: ImuNoise = ImuNoise(), antlever: np.ndarray = np.zeros((3,))):
        self.initstate = initstate
        self.initstate_std = initstate_std
        self.imunoise = imunoise
        self.antlever = antlever

    # def __init__(self):
    #     self.initstate = None
    #     self.initstate_std = None
    #     self.imunoise = None
    #     self.antlever = np.array([0,0,0])

    def print_options(self):
        print("---------------KF-GINS Options:---------------")

        # Print initial state
        print(" - Initial State: ")
        print('\t' + "- initial position: ", end="  ")
        print("{:.12f}".format(self.initstate.pos[0] * Angle.R2D), end="  ")
        print("{:.12f}".format(self.initstate.pos[1] * Angle.R2D), end="  ")
        print("{:.6f}".format(self.initstate.pos[2]), "[deg, deg, m]")

        print('\t' + "- initial velocity: ", self.initstate.vel, "[m/s]")
        print('\t' + "- initial attitude: ", self.initstate.euler * Angle.R2D, "[deg]")
        print('\t' + "- initial gyrbias: ", self.initstate.imuerror.gyrbias * Angle.R2D * 3600, "[deg/h]")
        print('\t' + "- initial accbias: ", self.initstate.imuerror.accbias * 1e5, "[mGal]")
        print('\t' + "- initial gyrscale: ", self.initstate.imuerror.gyrscale * 1e6, "[ppm]")
        print('\t' + "- initial accscale: ", self.initstate.imuerror.accscale * 1e6, "[ppm]")

        # Print initial state STD
        print(" - Initial State STD: ")
        print('\t' + "- initial position std: ", self.initstate_std.pos, "[m]")
        print('\t' + "- initial velocity std: ", self.initstate_std.vel, "[m/s]")
        print('\t' + "- initial attitude std: ", self.initstate_std.euler * Angle.R2D, "[deg]")
        print('\t' + "- initial gyrbias std: ", self.initstate_std.imuerror.gyrbias * Angle.R2D * 3600, "[deg/h]")
        print('\t' + "- initial accbias std: ", self.initstate_std.imuerror.accbias * 1e5, "[mGal]")
        print('\t' + "- initial gyrscale std: ", self.initstate_std.imuerror.gyrscale * 1e6, "[ppm]")
        print('\t' + "- initial accscale std: ", self.initstate_std.imuerror.accscale * 1e6, "[ppm]")


        # Print IMU noise parameters
        print(" - IMU noise: ")
        print('\t' + "- arw: ", self.imunoise.gyr_arw * Angle.R2D * 60, "[deg/sqrt(h)]")
        print('\t' + "- vrw: ", self.imunoise.acc_vrw * 60, "[m/s/sqrt(h)]")
        print('\t' + "- gyrbias std: ", self.imunoise.gyrbias_std * Angle.R2D * 3600, "[deg/h]")
        print('\t' + "- accbias std: ", self.imunoise.accbias_std * 1e5, "[mGal]")
        print('\t' + "- gyrscale std: ", self.imunoise.gyrscale_std * 1e6, "[ppm]")
        print('\t' + "- accscale std: ", self.imunoise.accscale_std * 1e6, "[ppm]")
        print('\t' + "- correlation time: ", self.imunoise.corr_time / 3600.0, "[h]")

        # Print GNSS antenna leverarm
        print(" - Antenna leverarm: ", self.antlever, "[m]")
        print()
