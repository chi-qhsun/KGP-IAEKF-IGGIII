from common.types import *
from common.earth import *
from common.rotation import *
from kf_gins_types import *

class INSMech:

    @staticmethod
    def insMech(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU):
        """
        INS机械编排算法, 利用IMU数据进行速度、位置和姿态更新
        INS Mechanization, update velocity, position and attitude using imudata

        Args:
            pvapre (PVA): 上一时刻状态 / the last imustate
            pvacur (PVA): 输出当前时刻状态 / output the current imustate
            imupre (IMU): imudata at the previous time step
            imucur (IMU): imudata at the current time step
        """
        cur = INSMech.velUpdate(pvapre, pvacur, imupre, imucur)
        cur = INSMech.posUpdate(pvapre, cur, imupre, imucur)
        cur = INSMech.attUpdate(pvapre, cur, imupre, imucur)
        return cur

    @staticmethod
    def posUpdate(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU):
        """
        位置更新
        position update

        Args:
            pvapre (PVA): 上一时刻状态 / the last imustate
            pvacur (PVA): 输出当前时刻状态 / output the current imustate
            imupre (IMU): imudata at the previous time step
            imucur (IMU): imudata at the current time step
        """
        midvel = (pvacur.vel + pvapre.vel) / 2
        midpos = pvapre.pos + Earth.DRi(pvapre.pos) * midvel * imucur.dt / 2

        rmrn = Earth.meridianPrimeVerticalRadius(midpos[0])
        wie_n = np.array([Earth.WGS84_WIE * np.cos(midpos[0]), 0, -Earth.WGS84_WIE * np.sin(midpos[0])])
        wen_n = np.array([midvel[1] / (rmrn[1] + midpos[2]),
                          -midvel[0] / (rmrn[0] + midpos[2]),
                          -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])])

        temp1 = -(wie_n + wen_n) * imucur.dt
        qnn = Rotation.rotvec2quaternion(temp1)

        temp2 = np.array([0, 0, -Earth.WGS84_WIE * imucur.dt])
        qee = Rotation.rotvec2quaternion(temp2)

        qne = Earth.qne(pvapre.pos)
        qne = np.dot(np.dot(qee, qne), qnn)
        cur = pvacur
        cur.pos[2] = pvapre.pos[2] - midvel[2] * imucur.dt
        cur.pos = Earth.blh(qne, cur.pos[2])
        return cur


    @staticmethod
    def velUpdate(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU):
        """
        速度更新
        velocity update

        Args:
            pvapre (PVA): 上一时刻状态 / the last imustate
            pvacur (PVA): 输出当前时刻状态 / output the current imustate
            imupre (IMU): imudata at the previous time step
            imucur (IMU): imudata at the current time step
        """
        rmrn = Earth.meridianPrimeVerticalRadius(pvapre.pos[0])
        wie_n = np.array([Earth.WGS84_WIE * np.cos(pvapre.pos[0]), 0, -Earth.WGS84_WIE * np.sin(pvapre.pos[0])])
        wen_n = np.array([pvapre.vel[1] / (rmrn[1] + pvapre.pos[2]),
                          -pvapre.vel[0] / (rmrn[0] + pvapre.pos[2]),
                          -pvapre.vel[1] * np.tan(pvapre.pos[0]) / (rmrn[1] + pvapre.pos[2])])
        gravity = Earth.gravity(pvapre.pos)

        temp1 = np.cross(imucur.dtheta, imucur.dvel) / 2
        temp2 = np.cross(imupre.dtheta, imucur.dvel) / 12
        temp3 = np.cross(imupre.dvel, imucur.dtheta) / 12

        d_vfb = imucur.dvel + temp1 + temp2 + temp3

        temp1 = (wie_n + wen_n) * imucur.dt / 2
        cnn = np.eye(3) - Rotation.skewSymmetric(temp1)
        d_vfn = np.dot(cnn, np.dot(pvapre.att.cbn, d_vfb))

        gl = np.array([0, 0, gravity])
        d_vgn = (gl - np.cross(2 * wie_n + wen_n, pvapre.vel)) * imucur.dt

        midvel = pvapre.vel + (d_vfn + d_vgn) / 2

        qnn = Rotation.rotvec2quaternion(temp1)
        temp2 = np.array([0, 0, -Earth.WGS84_WIE * imucur.dt / 2])
        qee = Rotation.rotvec2quaternion(temp2)
        qne = Earth.qne(pvapre.pos)
        qne = np.dot(np.dot(qee, qne), qnn)
        midpos = np.array(pvapre.pos)
        midpos[2] = pvapre.pos[2] - midvel[2] * imucur.dt / 2
        midpos = Earth.blh(qne, midpos[2])

        rmrn = Earth.meridianPrimeVerticalRadius(midpos[0])
        wie_n = np.array([Earth.WGS84_WIE * np.cos(midpos[0]), 0, -Earth.WGS84_WIE * np.sin(midpos[0])])
        wen_n = np.array([midvel[1] / (rmrn[1] + midpos[2]),
                          -midvel[0] / (rmrn[0] + midpos[2]),
                          -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])])

        temp3 = (wie_n + wen_n) * imucur.dt / 2
        cnn = np.eye(3) - Rotation.skewSymmetric(temp3)
        d_vfn = np.dot(cnn, np.dot(pvapre.att.cbn, d_vfb))

        gl = np.array([0, 0, Earth.gravity(midpos)])
        d_vgn = (gl - np.cross(2 * wie_n + wen_n, midvel)) * imucur.dt
        cur = pvacur
        cur.vel = pvapre.vel + d_vfn + d_vgn
        return cur

    @staticmethod
    def attUpdate(pvapre: PVA, pvacur: PVA, imupre: IMU, imucur: IMU):
        """
        姿态更新
        attitude update

        Args:
            pvapre (PVA): 上一时刻状态 / the last imustate
            pvacur (PVA): 输出当前时刻状态 / output the current imustate
            imupre (IMU): imudata at the previous time step
            imucur (IMU): imudata at the current time step
        """
        midvel = (pvapre.vel + pvacur.vel) / 2
        qne_pre = Earth.qne(pvapre.pos)
        qne_cur = Earth.qne(pvacur.pos)
        temp1 = Rotation.quaternion2vector(qne_cur.inverse() * qne_pre)
        qne_mid = qne_pre * Rotation.rotvec2quaternion(temp1 / 2).inverse()
        midpos_2 = (pvacur.pos[2] + pvapre.pos[2]) / 2
        midpos = Earth.blh(qne_mid, midpos_2)

        rmrn = Earth.meridianPrimeVerticalRadius(midpos[0])
        wie_n = np.array([Earth.WGS84_WIE * np.cos(midpos[0]), 0, -Earth.WGS84_WIE * np.sin(midpos[0])])
        wen_n = np.array([midvel[1] / (rmrn[1] + midpos[2]), -midvel[0] / (rmrn[0] + midpos[2]),
                          -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])])

        temp1 = -(wie_n + wen_n) * imucur.dt
        qnn = Rotation.rotvec2quaternion(temp1)

        temp1 = imucur.dtheta + np.cross(imupre.dtheta, imucur.dtheta) / 12
        qbb = Rotation.rotvec2quaternion(temp1)
        cur = pvacur
        cur.att.qbn = qnn * pvapre.att.qbn * qbb
        cur.att.cbn = Rotation.quaternion2matrix(cur.att.qbn)
        cur.att.euler = Rotation.matrix2euler(cur.att.cbn)
        return cur
