from common.angle import *
import numpy as np
from pyquaternion import Quaternion

class Attitude:
    def __init__(self):
        self.qbn = Quaternion(0,0,0,0)
        self.cbn = np.zeros((3,3))
        self.euler = np.zeros((3,))

class PVA:
    def __init__(self):
        self.pos = np.zeros((3,))
        self.vel = np.zeros((3,))
        self.att = Attitude()

class ImuError:
    def __init__(self):
        self.gyrbias = np.zeros((3,))
        self.accbias = np.zeros((3,))
        self.gyrscale = np.zeros((3,))
        self.accscale = np.zeros((3,))

class NavState:
    def __init__(self):
        self.pos = np.zeros((3,))
        self.vel = np.zeros((3,))
        self.euler = np.zeros((3,))
        self.imuerror = ImuError()

class ImuNoise:
    def __init__(self):
        self.gyr_arw = np.zeros((3,))
        self.acc_vrw = np.zeros((3,))
        self.gyrbias_std = np.zeros((3,))
        self.accbias_std = np.zeros((3,))
        self.gyrscale_std = np.zeros((3,))
        self.accscale_std = np.zeros((3,))
        self.corr_time = 0.0

class GINSOptions:
    def __init__(self):
        self.initstate = NavState()
        self.initstate_std = NavState()
        self.imunoise = ImuNoise()
        self.antlever = np.zeros((3,))
        self.kgp_status = False

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
