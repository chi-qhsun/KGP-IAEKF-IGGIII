from common.types import *
from .kf_gins_types import *

from common.earth import *
from common.rotation import *
from .insmech import *
import numpy as np
import sys

class GIEngine:
    class StateID:
        P_ID = 0
        V_ID = 3
        PHI_ID = 6
        BG_ID = 9
        BA_ID = 12
        SG_ID = 15
        SA_ID = 18

    class NoiseID:
        VRW_ID = 0
        ARW_ID = 3
        BGSTD_ID = 6
        BASTD_ID = 9
        SGSTD_ID = 12
        SASTD_ID = 15

    def __init__(self, options: GINSOptions):



        # 更新时间对齐误差，IMU状态和观测信息误差小于它则认为两者对齐
        # updata time align error
        self.TIME_ALIGN_ERR = 0.001

        # IMU和GNSS原始数据
        # raw imudata and gnssdata
        self.imupre_ = IMU()
        self.imucur_ = IMU()
        self.gnssdata_ = GNSS()

        # # IMU状态（位置、速度、姿态和IMU误差）
        # # imu state (position, velocity, attitude and imu error)
        self.pvacur_ = PVA()
        self.pvapre_ = PVA()
        self.imuerror_ = ImuError()

        self.RANK = 21
        self.NOISERANK = 18
        # -------------------------------------------------------------

        self.options_ = options
        self.options_.print_options()
        self.timestamp_ = 0

        # Resize covariance matrix, system noise matrix, and system error state matrix
        self.Cov_ = np.zeros((self.RANK, self.RANK))
        self.Qc_ = np.zeros((self.NOISERANK, self.NOISERANK))
        self.dx_ = np.zeros((self.RANK, 1))

        # Initialize noise matrix
        imunoise = self.options_.imunoise
        self.Qc_[GIEngine.NoiseID.ARW_ID:GIEngine.NoiseID.ARW_ID + 3, GIEngine.NoiseID.ARW_ID:GIEngine.NoiseID.ARW_ID + 3] = np.diag(imunoise.gyr_arw ** 2)
        self.Qc_[GIEngine.NoiseID.VRW_ID:GIEngine.NoiseID.VRW_ID + 3, GIEngine.NoiseID.VRW_ID:GIEngine.NoiseID.VRW_ID + 3] = np.diag(imunoise.acc_vrw ** 2)
        self.Qc_[GIEngine.NoiseID.BGSTD_ID:GIEngine.NoiseID.BGSTD_ID + 3, GIEngine.NoiseID.BGSTD_ID:GIEngine.NoiseID.BGSTD_ID + 3] = 2 / imunoise.corr_time * np.diag(imunoise.gyrbias_std ** 2)
        self.Qc_[GIEngine.NoiseID.BASTD_ID:GIEngine.NoiseID.BASTD_ID + 3, GIEngine.NoiseID.BASTD_ID:GIEngine.NoiseID.BASTD_ID + 3] = 2 / imunoise.corr_time * np.diag(imunoise.accbias_std ** 2)
        self.Qc_[GIEngine.NoiseID.SGSTD_ID:GIEngine.NoiseID.SGSTD_ID + 3, GIEngine.NoiseID.SGSTD_ID:GIEngine.NoiseID.SGSTD_ID + 3] = 2 / imunoise.corr_time * np.diag(imunoise.gyrscale_std ** 2)
        self.Qc_[GIEngine.NoiseID.SASTD_ID:GIEngine.NoiseID.SASTD_ID + 3, GIEngine.NoiseID.SASTD_ID:GIEngine.NoiseID.SASTD_ID + 3] = 2 / imunoise.corr_time * np.diag(imunoise.accscale_std ** 2)

        # Set initial state and covariance
        self.initialize(self.options_.initstate, self.options_.initstate_std)



    def initialize(self, initstate: NavState, initstate_std: NavState):
        # Initialize position, velocity, and attitude
        # self.pvacur_ = PVA()
        self.pvacur_.pos = initstate.pos
        self.pvacur_.vel = initstate.vel
        self.pvacur_.att.euler = initstate.euler
        # print('self.pvacur_.att.euler init', self.pvacur_.att.euler) # todo:
        self.pvacur_.att.cbn = Rotation.euler2matrix(self.pvacur_.att.euler)
        # print('self.pvacur_.att.cbn init',self.pvacur_.att.cbn)
        self.pvacur_.att.qbn = Rotation.euler2quaternion(self.pvacur_.att.euler)

        # Initialize IMU error
        self.imuerror_ = initstate.imuerror

        # Set the same value to the previous state
        # self.pvapre_ = self.pvacur_
        self.pvapre_.pos = self.pvacur_.pos
        self.pvapre_.vel = self.pvacur_.vel
        self.pvapre_.att.euler = self.pvacur_.att.euler
        self.pvapre_.att.cbn = self.pvacur_.att.cbn
        self.pvapre_.att.qbn = self.pvacur_.att.qbn

        # Initialize covariance
        imuerror_std = initstate_std.imuerror
        self.Cov_[GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3, GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3] = np.diag(initstate_std.pos ** 2)
        self.Cov_[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3] = np.diag(initstate_std.vel ** 2)
        self.Cov_[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3] = np.diag(initstate_std.euler ** 2)
        self.Cov_[GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3, GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3] = np.diag(imuerror_std.gyrbias ** 2)
        self.Cov_[GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3, GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3] = np.diag(imuerror_std.accbias ** 2)
        self.Cov_[GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3, GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3] = np.diag(imuerror_std.gyrscale ** 2)
        self.Cov_[GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3, GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3] = np.diag(imuerror_std.accscale ** 2)


    def newImuProcess(self):
        # Current IMU time as the current state time
        self.timestamp_ = self.imucur_.time

        # If GNSS is valid, set update time as the GNSS time
        updatetime = self.gnssdata_.time if self.gnssdata_.isvalid else -1
        # print('updatetime: ',updatetime)
        # Determine if GNSS update is needed
        # print('pre_: ',self.imupre_.time, 'cur_:', self.imucur_.time)
        res = self.isToUpdate(self.imupre_.time, self.imucur_.time, updatetime)
        # print('res: ',res)
        if res == 0:
            # Only propagate navigation state
            self.insPropagation(self.imupre_, self.imucur_)
        elif res == 1:
            # GNSS data is near to the previous imudata, we should firstly do gnss update
            self.gnssdata_ = self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()
            self.pvapre_ = self.pvacur_
            self.insPropagation(self.imupre_, self.imucur_)
        elif res == 2:
            # GNSS data is near current imudata, we should firstly propagate navigation state
            self.insPropagation(self.imupre_, self.imucur_)
            self.gnssdata_ = self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()
        elif res == 3:
            # GNSS data is between the two imudata, we interpolate current imudata to gnss time
            midimu = IMU()  # Define midimu as needed
            midimu, self.imucur_ = self.imuInterpolate(self.imupre_, self.imucur_, updatetime, midimu)

            # Propagate navigation state for the first half imudata
            self.insPropagation(self.imupre_, midimu)

            # Do GNSS position update at the whole second and feedback system states
            self.gnssdata_ = self.gnssUpdate(self.gnssdata_)
            self.stateFeedback()

            # Propagate navigation state for the second half imudata
            self.pvapre_ = self.pvacur_
            self.insPropagation(midimu, self.imucur_)

            # Check diagonal elements of current covariance matrix
        self.checkCov()

        # Update system state and imudata at the previous epoch
        # self.pvapre_ = self.pvacur_
        self.pvapre_.pos = self.pvacur_.pos
        self.pvapre_.vel = self.pvacur_.vel
        self.pvapre_.att.euler = self.pvacur_.att.euler
        self.pvapre_.att.cbn = self.pvacur_.att.cbn
        self.pvapre_.att.qbn = self.pvacur_.att.qbn

        # self.imupre_ = self.imucur_
        self.imupre_.time = self.imucur_.time
        self.imupre_.dt = self.imucur_.dt
        self.imupre_.dtheta = self.imucur_.dtheta
        self.imupre_.dvel = self.imucur_.dvel
        self.imupre_.odovel = self.imucur_.odovel

    def imuCompensate(self, imu: IMU):
        # imunew = imu
        imunew = IMU()
        imunew.time = imu.time
        imunew.dt = imu.dt
        imunew.dtheta = imu.dtheta
        imunew.dvel = imu.dvel
        imunew.odovel = imu.odovel


        # Compensate the IMU bias
        imunew.dtheta -= self.imuerror_.gyrbias * imu.dt
        imunew.dvel -= self.imuerror_.accbias * imu.dt

        # Compensate the IMU scale
        gyrscale = np.ones(3) + self.imuerror_.gyrscale
        accscale = np.ones(3) + self.imuerror_.accscale

        imunew.dtheta *= gyrscale ** -1
        imunew.dvel *= accscale ** -1
        return imunew

    def insPropagation(self, imupre: IMU, imucur: IMU):
        # Compensate IMU error to 'imucur', 'imupre' has been compensated
        imucur = self.imuCompensate(imucur)

        # Update IMU state (mechanization)
        # print('pvacru_ vel before insmech:', self.pvacur_.vel, self.pvapre_.vel)
        self.pvacur_ = INSMech.insMech(self.pvapre_, self.pvacur_, imupre, imucur)
        # print('pvacru_ vel after insmech:', self.pvacur_.vel, self.pvapre_.vel)

        # System noise propagate, phi-angle error model for attitude error
        Phi = np.zeros_like(self.Cov_)
        F = np.zeros_like(self.Cov_)
        Qd = np.zeros_like(self.Cov_)
        G = np.zeros((self.RANK, self.NOISERANK))

        # Initialize matrices
        Phi = np.eye(Phi.shape[0])
        F = np.zeros_like(F)
        Qd = np.zeros_like(Qd)
        G = np.zeros((self.RANK, self.NOISERANK))

        # Compute state transition matrix using the previous state
        rmrn = Earth.meridianPrimeVerticalRadius(self.pvapre_.pos[0])
        gravity = Earth.gravity(self.pvapre_.pos)
        wie_n = np.array([Earth.WGS84_WIE * np.cos(self.pvapre_.pos[0]), 0, -Earth.WGS84_WIE * np.sin(self.pvapre_.pos[0])])
        wen_n = np.array([self.pvapre_.vel[1] / (rmrn[1] + self.pvapre_.pos[2]), -self.pvapre_.vel[0] / (rmrn[0] + self.pvapre_.pos[2]),
                          -self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0]) / (rmrn[1] + self.pvapre_.pos[2])])

        temp = np.zeros((3, 3))
        rmh = rmrn[0] + self.pvapre_.pos[2]
        rnh = rmrn[1] + self.pvapre_.pos[2]
        accel = imucur.dvel / imucur.dt
        omega = imucur.dtheta / imucur.dt

        # Position error
        temp[0, 0] = -self.pvapre_.vel[2] / rmh
        temp[0, 2] = self.pvapre_.vel[0] / rmh
        temp[1, 0] = self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0]) / rnh
        temp[1, 1] = -(self.pvapre_.vel[2] + self.pvapre_.vel[0] * np.tan(self.pvapre_.pos[0])) / rnh
        temp[1, 2] = self.pvapre_.vel[1] / rnh
        F[GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3, GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3] = temp
        F[GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3, GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3] = np.eye(3)

        # Velocity error
        temp[0, 0] = -2 * self.pvapre_.vel[1] * Earth.WGS84_WIE * np.cos(self.pvapre_.pos[0]) / rmh - self.pvapre_.vel[1] ** 2 / (rmh * rnh * np.cos(self.pvapre_.pos[0]) ** 2)
        temp[0, 2] = self.pvapre_.vel[0] * self.pvapre_.vel[2] / rmh / rmh - (self.pvapre_.vel[1]) ** 2 * np.tan(self.pvapre_.pos[0]) / rnh / rnh
        temp[1, 0] = 2 * Earth.WGS84_WIE * (self.pvapre_.vel[0] * np.cos(self.pvapre_.pos[0]) - self.pvapre_.vel[2] * np.sin(self.pvapre_.pos[0])) / rmh + self.pvapre_.vel[0] * self.pvapre_.vel[1] / rmh / rnh / np.cos(self.pvapre_.pos[0]) ** 2
        temp[1, 2] = (self.pvapre_.vel[1] * self.pvapre_.vel[2] + self.pvapre_.vel[0] * self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0])) / rnh / rnh
        temp[2, 0] = 2 * Earth.WGS84_WIE * self.pvapre_.vel[1] * np.sin(self.pvapre_.pos[0]) / rmh
        temp[2, 2] = -self.pvapre_.vel[1] ** 2 / rnh / rnh - self.pvapre_.vel[0] ** 2 / rmh / rmh + 2 * gravity / (np.sqrt(rmrn[0] * rmrn[1]) + self.pvapre_.pos[2])
        F[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3] = temp
        temp = np.zeros((3, 3))
        temp[0, 0] = self.pvapre_.vel[2] / rmh
        temp[0, 1] = -2 * (Earth.WGS84_WIE * np.sin(self.pvapre_.pos[0]) + self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0]) / rnh)
        temp[0, 2] = self.pvapre_.vel[0] / rmh
        temp[1, 0] = 2 * Earth.WGS84_WIE * np.sin(self.pvapre_.pos[0]) + self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0]) / rnh
        temp[1, 1] = (self.pvapre_.vel[2] + self.pvapre_.vel[0] * np.tan(self.pvapre_.pos[0])) / rnh
        temp[1, 2] = 2 * Earth.WGS84_WIE * np.cos(self.pvapre_.pos[0]) + self.pvapre_.vel[1] / rnh
        temp[2, 0] = -2 * self.pvapre_.vel[0] / rmh
        temp[2, 1] = -2 * (Earth.WGS84_WIE * np.cos(self.pvapre_.pos[0]) + self.pvapre_.vel[1] / rnh)
        F[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3] = temp
        F[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3] = Rotation.skewSymmetric(np.dot(self.pvapre_.att.cbn, accel))
        F[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3] = self.pvapre_.att.cbn
        F[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3] = np.dot(self.pvapre_.att.cbn, np.diag(accel))

        # Attitude error
        temp = np.zeros((3, 3))
        temp[0, 0] = -Earth.WGS84_WIE * np.sin(self.pvapre_.pos[0]) / rmh
        temp[0, 2] = self.pvapre_.vel[1] / rnh / rnh
        temp[1, 2] = -self.pvapre_.vel[0] / rmh / rmh
        temp[2, 0] = -Earth.WGS84_WIE * np.cos(self.pvapre_.pos[0]) / rmh - self.pvapre_.vel[1] / (rmh * rnh * np.cos(self.pvapre_.pos[0]) ** 2)
        temp[2, 2] = -self.pvapre_.vel[1] * np.tan(self.pvapre_.pos[0]) / rnh / rnh
        F[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3] = temp
        temp = np.zeros((3, 3))
        temp[0, 1] = 1 / rnh
        temp[1, 0] = -1 / rmh
        temp[2, 1] = -np.tan(self.pvapre_.pos[0]) / rnh
        F[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3] = temp
        F[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3] = -Rotation.skewSymmetric(wie_n + wen_n)
        F[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3] = -self.pvapre_.att.cbn
        F[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3] = -np.dot(self.pvapre_.att.cbn, np.diag(omega))

        # IMU bias error and scale error, modeled as the first-order Gauss-Markov process
        F[GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3, GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3, GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3, GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3] = -1 / self.options_.imunoise.corr_time * np.eye(3)
        F[GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3, GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3] = -1 / self.options_.imunoise.corr_time * np.eye(3)

        # System noise driven matrix
        G[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, GIEngine.NoiseID.VRW_ID:GIEngine.NoiseID.VRW_ID + 3] = self.pvapre_.att.cbn
        G[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, GIEngine.NoiseID.ARW_ID:GIEngine.NoiseID.ARW_ID + 3] = self.pvapre_.att.cbn
        G[GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3, GIEngine.NoiseID.BGSTD_ID:GIEngine.NoiseID.BGSTD_ID + 3] = np.eye(3)
        G[GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3, GIEngine.NoiseID.BASTD_ID:GIEngine.NoiseID.BASTD_ID + 3] = np.eye(3)
        G[GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3, GIEngine.NoiseID.SGSTD_ID:GIEngine.NoiseID.SGSTD_ID + 3] = np.eye(3)
        G[GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3, GIEngine.NoiseID.SASTD_ID:GIEngine.NoiseID.SASTD_ID + 3] = np.eye(3)

        # Compute the state transition matrix
        Phi = np.eye(Phi.shape[0]) + F * imucur.dt

        # Compute system propagation noise
        Qd = np.dot(G, np.dot(self.Qc_, np.dot(G.transpose(), imucur.dt)))
        Qd = (np.dot(Phi, np.dot(Qd, Phi.transpose())) + Qd) / 2

        # Do EKF predict to propagate covariance and error state
        self.EKFPredict(Phi, Qd)

    def gnssUpdate(self, gnssdata: GNSS):
        # Convert IMU position to GNSS antenna phase center position
        Dr_inv = Earth.DRi(self.pvacur_.pos)
        Dr = Earth.DR(self.pvacur_.pos)
        antenna_pos = self.pvacur_.pos + Dr_inv @ self.pvacur_.att.cbn @ self.options_.antlever

        # Compute GNSS position innovation
        dz = Dr @ (antenna_pos - gnssdata.blh)
        dz = dz.reshape((-1,1))
        # Construct GNSS position measurement matrix
        H_gnsspos = np.zeros((3, self.Cov_.shape[0]))
        H_gnsspos[0:3, GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3] = np.eye(3)
        H_gnsspos[0:3, GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3] = Rotation.skewSymmetric(self.pvacur_.att.cbn @ self.options_.antlever)

        # Construct measurement noise matrix
        R_gnsspos = np.diag(gnssdata.std ** 2)

        # Do EKF update to update covariance and error state
        self.EKFUpdate(dz, H_gnsspos, R_gnsspos)

        # Set GNSS invalid after update
        # gnssdatanew = gnssdata
        gnssdatanew = GNSS()
        gnssdatanew.time = gnssdata.time
        gnssdatanew.blh = gnssdata.blh
        gnssdatanew.std = gnssdata.std
        gnssdatanew.isvalid = gnssdata.isvalid

        gnssdatanew.isvalid = False
        return gnssdatanew

    def isToUpdate(self, imutime1: float, imutime2: float, updatetime: float):
        # print(imutime1,imutime2,updatetime,abs(imutime2 - updatetime))
        if abs(imutime1 - updatetime) < self.TIME_ALIGN_ERR:
            return 1  # updatetime is near to imutime1
        elif abs(imutime2 - updatetime) <= self.TIME_ALIGN_ERR:
            return 2  # updatetime is near to imutime2
        elif imutime1 < updatetime < imutime2:
            return 3  # updatetime is between imutime1 and imutime2, but not near to either
        else:
            return 0  # updatetime is not between imutime1 and imutime2, and not near to either

    def EKFPredict(self, Phi: np.ndarray, Qd: np.ndarray):
        assert Phi.shape[0] == self.Cov_.shape[0]
        assert Qd.shape[0] == self.Cov_.shape[0]

        # propagate system covariance and error state
        self.Cov_ = Phi @ self.Cov_ @ Phi.T + Qd
        self.dx_ = Phi @ self.dx_

    def EKFUpdate(self, dz: np.ndarray, H: np.ndarray, R: np.ndarray):
        assert H.shape[1] == self.Cov_.shape[0]
        assert dz.shape[0] == H.shape[0]
        assert dz.shape[0] == R.shape[0]
        assert dz.shape[1] == 1

        # Compute Kalman Gain
        temp = H @ self.Cov_ @ H.T + R
        K = self.Cov_ @ H.T @ np.linalg.inv(temp)

        # update system error state and covariance
        I = np.identity(self.Cov_.shape[0])
        I = I - K @ H
        self.dx_ = self.dx_ + K @ (dz - H @ self.dx_)
        self.Cov_ = I @ self.Cov_ @ I.T + K @ R @ K.T

    def stateFeedback(self):

        # Position error feedback
        delta_r = self.dx_[GIEngine.StateID.P_ID:GIEngine.StateID.P_ID + 3, 0]
        Dr_inv = Earth.DRi(self.pvacur_.pos)
        self.pvacur_.pos -= np.dot(Dr_inv, delta_r)

        # Velocity error feedback
        vectemp = self.dx_[GIEngine.StateID.V_ID:GIEngine.StateID.V_ID + 3, 0]
        self.pvacur_.vel -= vectemp

        # Attitude error feedback
        vectemp = self.dx_[GIEngine.StateID.PHI_ID:GIEngine.StateID.PHI_ID + 3, 0]
        qpn = Rotation.rotvec2quaternion(vectemp)
        self.pvacur_.att.qbn = qpn * self.pvacur_.att.qbn
        self.pvacur_.att.cbn = Rotation.quaternion2matrix(self.pvacur_.att.qbn)
        self.pvacur_.att.euler = Rotation.matrix2euler(self.pvacur_.att.cbn)

        # IMU bias error feedback
        vectemp = self.dx_[GIEngine.StateID.BG_ID:GIEngine.StateID.BG_ID + 3, 0]
        self.imuerror_.gyrbias += vectemp
        vectemp = self.dx_[GIEngine.StateID.BA_ID:GIEngine.StateID.BA_ID + 3, 0]
        self.imuerror_.accbias += vectemp

        # IMU scale error feedback
        vectemp = self.dx_[GIEngine.StateID.SG_ID:GIEngine.StateID.SG_ID + 3, 0]
        self.imuerror_.gyrscale += vectemp
        vectemp = self.dx_[GIEngine.StateID.SA_ID:GIEngine.StateID.SA_ID + 3, 0]
        self.imuerror_.accscale += vectemp

        # Set 'dx' to zero after feedback error state to system state
        self.dx_ = np.zeros_like(self.dx_)

    def getNavState(self):
        state = NavState()
        state.pos = self.pvacur_.pos
        state.vel = self.pvacur_.vel
        state.euler = self.pvacur_.att.euler
        state.imuerror = self.imuerror_
        return state

    def addImuData(self, imu: IMU, compensate=False):
        """
        Add new imudata and optionally compensate imu error.

        Args:
        imu (IMU): New raw imudata.
        compensate (bool, optional): Whether to compensate imu error to new imudata. Defaults to False.
        """
        # tmp = self.imucur_
        # self.imucur_ = None
        # self.imupre_ = tmp
        self.imupre_.time = self.imucur_.time
        self.imupre_.dt = self.imucur_.dt
        self.imupre_.dtheta = self.imucur_.dtheta
        self.imupre_.dvel = self.imucur_.dvel
        self.imupre_.odovel = self.imucur_.odovel

        # tmp = imu
        # imu = None
        # self.imucur_ = tmp
        self.imucur_.time = imu.time
        self.imucur_.dt = imu.dt
        self.imucur_.dtheta = imu.dtheta
        self.imucur_.dvel = imu.dvel
        self.imucur_.odovel = imu.odovel


        if compensate:
            self.imucur_ = self.imuCompensate(self.imucur_)

    def addGnssData(self, gnss: GNSS):
        """
        Add new gnssdata.

        Args:
        gnss (GNSS): New gnssdata.
        """
        self.gnssdata_.time = gnss.time

        self.gnssdata_.blh = gnss.blh
        self.gnssdata_.std = gnss.std

        # Do not check the validity of gnssdata, the gnssdata is valid by default
        self.gnssdata_.isvalid = True

    def imuInterpolate(self, imu1: IMU, imu2: IMU, timestamp:float, midimu: IMU):
        """
        Interpolate incremental imudata to given timestamp.

        Args:
        imu1 (IMU): The previous imudata.
        imu2 (IMU): The current imudata.
        timestamp (float): Given interpolate timestamp.
        Returns:
        IMU: Output imudata at given timestamp.
        """

        if imu1.time > timestamp or imu2.time < timestamp:
            return midimu, imu2

        lamda = (timestamp - imu1.time) / (imu2.time - imu1.time)

        mid = IMU()
        mid.time = timestamp
        mid.dtheta = imu2.dtheta * lamda
        mid.dvel = imu2.dvel * lamda
        mid.dt = timestamp - imu1.time
        mid.odovel = midimu.odovel

        # imu2new = imu2
        imu2new = IMU()
        imu2new.time = imu2.time
        imu2new.dt = imu2.dt
        imu2new.dtheta = imu2.dtheta
        imu2new.dvel = imu2.dvel
        imu2new.odovel = imu2.odovel

        imu2new.dtheta -= mid.dtheta
        imu2new.dvel -= mid.dvel
        imu2new.dt -= mid.dt

        return mid,imu2new

    def timestamp(self):
        """
        Get current time.
        """
        return self.timestamp_

    def getCovariance(self):
        """
        Get the covariance matrix.

        Returns:
        numpy.ndarray: The covariance matrix.
        """
        # Assuming Cov_ is a NumPy array
        return self.Cov_

    def checkCov(self):
        """
        Check if covariance diagonal elements are all positive.

        Parameters:
        Cov (numpy.ndarray): Covariance matrix.
        """
        for i in range(self.RANK):
            if self.Cov_[i, i] < 0:
                timestamp = self.timestamp_  # Assuming timestamp_ is a global variable
                print(f"Covariance is negative at {timestamp} !")
                sys.exit("EXIT_FAILURE")