from common.types import *
from common.earth import *
from common.rotation import *
from .kf_gins_types import *


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
        # print('pvacru_ before vel:', pvacur.pos*Angle.R2D, pvacur.vel, pvacur.att.euler)
        cur1 = INSMech.velUpdate(pvapre, pvacur, imupre, imucur)
        # print('pvacru_ after vel:', cur1.pos*Angle.R2D, cur1.vel,pvacur.att.euler)

        # print('pvacru_ before pos:', cur1.pos*Angle.R2D, cur1.vel,pvacur.att.euler)
        cur2 = INSMech.posUpdate(pvapre, cur1, imupre, imucur)
        # print('pvacru_ after pos:', cur2.pos*Angle.R2D, cur2.vel,pvacur.att.euler)

        # print('pvacru_ before att:', cur2.pos*Angle.R2D, cur2.vel,pvacur.att.euler)
        cur = INSMech.attUpdate(pvapre, cur2, imupre, imucur)
        # print('pvacru_ after att:', cur.pos*Angle.R2D, cur.vel,pvacur.att.euler)
        # print()

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
        # print('rmrn:',rmrn)
        wie_n = np.array([Earth.WGS84_WIE * np.cos(pvapre.pos[0]), 0, -Earth.WGS84_WIE * np.sin(pvapre.pos[0])])
        # print('wie_n:', wie_n)
        wen_n = np.array([pvapre.vel[1] / (rmrn[1] + pvapre.pos[2]),
                          -pvapre.vel[0] / (rmrn[0] + pvapre.pos[2]),
                          -pvapre.vel[1] * np.tan(pvapre.pos[0]) / (rmrn[1] + pvapre.pos[2])])
        # print('wen_n:', wen_n)
        gravity = Earth.gravity(pvapre.pos)
        # print('gravity:', gravity)

        temp1 = np.cross(imucur.dtheta, imucur.dvel) / 2
        temp2 = np.cross(imupre.dtheta, imucur.dvel) / 12
        temp3 = np.cross(imupre.dvel, imucur.dtheta) / 12
        # print('temp:', temp1, ',', temp2, ',', temp3)

        d_vfb = imucur.dvel + temp1 + temp2 + temp3
        # print('d_vfb:', d_vfb)

        temp1 = (wie_n + wen_n) * imucur.dt / 2
        # print('temp1:', temp1)
        cnn = np.eye(3) - Rotation.skewSymmetric(temp1)
        # print('cnn:', cnn)
        d_vfn = cnn @ pvapre.att.cbn @ d_vfb
        # print("pvapre.att.cbn:",pvapre.att.cbn)
        # print('d_vfn:', d_vfn)

        gl = np.array([0, 0, gravity])
        # print('gl:', gl)
        d_vgn = (gl - np.cross((2 * wie_n + wen_n), pvapre.vel)) * imucur.dt
        # print('d_vgn:', d_vgn)

        midvel = pvapre.vel + (d_vfn + d_vgn) / 2
        # print('midvel:', midvel)

        qnn = Rotation.rotvec2quaternion(temp1)
        # print('qnn:', qnn.w, qnn.x, qnn.y, qnn.z)
        # print()
        temp2 = np.array([0, 0, -Earth.WGS84_WIE * imucur.dt / 2])
        # print('temp2:', temp2)
        qee = Rotation.rotvec2quaternion(temp2)
        # print('qee:', qee)
        qne = Earth.qne(pvapre.pos)
        # print('qne:', qne)
        qne = qee * qne * qnn
        # print('qne:', qne)

        # midpos = np.array(pvapre.pos)
        midpos_2 = pvapre.pos[2] - midvel[2] * imucur.dt / 2
        # print('midpos_2:', midpos_2)
        midpos = Earth.blh(qne, midpos_2)
        # print('midpos:', midpos)

        rmrn = Earth.meridianPrimeVerticalRadius(midpos[0])
        # print('rmrn:', rmrn)
        wie_n = np.array([Earth.WGS84_WIE * np.cos(midpos[0]), 0, -Earth.WGS84_WIE * np.sin(midpos[0])])
        # print('wie_n:', wie_n)
        wen_n = np.array([midvel[1] / (rmrn[1] + midpos[2]),
                          -midvel[0] / (rmrn[0] + midpos[2]),
                          -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])])
        # print('wen_n:', wen_n)

        temp3 = (wie_n + wen_n) * imucur.dt / 2
        # print('temp3:', temp3)
        cnn = np.eye(3) - Rotation.skewSymmetric(temp3)
        # print('cnn:', cnn)

        d_vfn = cnn @ pvapre.att.cbn @ d_vfb
        # print('d_vfn:', d_vfn)

        gl = np.array([0, 0, Earth.gravity(midpos)])
        # print('gl:', gl)
        d_vgn = (gl - np.cross((2 * wie_n + wen_n), midvel)) * imucur.dt
        # print('d_vgn:', d_vgn)

        # cur = pvacur
        cur = PVA()
        cur.pos = pvacur.pos
        cur.vel = pvacur.vel
        # cur.att = pvacur.att
        cur.att.qbn = Quaternion(pvacur.att.qbn)
        cur.att.cbn = pvacur.att.cbn
        cur.att.euler = pvacur.att.euler

        cur.vel = pvapre.vel + d_vfn + d_vgn
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
        midpos = pvapre.pos + Earth.DRi(pvapre.pos) @ midvel * imucur.dt / 2

        rmrn = Earth.meridianPrimeVerticalRadius(midpos[0])
        wie_n = np.array([Earth.WGS84_WIE * np.cos(midpos[0]), 0, -Earth.WGS84_WIE * np.sin(midpos[0])])
        wen_n = np.array([midvel[1] / (rmrn[1] + midpos[2]),
                          -midvel[0] / (rmrn[0] + midpos[2]),
                          -midvel[1] * np.tan(midpos[0]) / (rmrn[1] + midpos[2])])

        temp1 = (wie_n + wen_n) * imucur.dt
        # print('temp1',temp1)
        qnn = Rotation.rotvec2quaternion(temp1)
        # print('qnn:', qnn.w, qnn.x, qnn.y, qnn.z)

        temp2 = np.array([0, 0, -Earth.WGS84_WIE * imucur.dt])
        qee = Rotation.rotvec2quaternion(temp2)
        # print('qee:', qee.w, qee.x, qee.y, qee.z)

        qne = Earth.qne(pvapre.pos)
        # print('qne:', qne.w, qne.x, qne.y, qne.z)
        qne = qee * qne * qnn
        # print('qne:', qne.w, qne.x, qne.y, qne.z)

        # cur = pvacur
        cur = PVA()
        cur.pos = pvacur.pos
        cur.vel = pvacur.vel
        # cur.att = pvacur.att
        cur.att.qbn = Quaternion(pvacur.att.qbn)
        cur.att.cbn = pvacur.att.cbn
        cur.att.euler = pvacur.att.euler

        cur.pos[2] = pvapre.pos[2] - midvel[2] * imucur.dt
        cur.pos = Earth.blh(qne, cur.pos[2])
        print('cur.pos:', cur.pos)

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

        temp1 = Rotation.quaternion2vector(qne_cur.inverse * qne_pre)

        qne_mid = qne_pre * Rotation.rotvec2quaternion(temp1 / 2).inverse
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
        # cur = pvacur
        cur = PVA()
        cur.pos = pvacur.pos
        cur.vel = pvacur.vel
        # cur.att = pvacur.att
        cur.att.qbn = Quaternion(pvacur.att.qbn)
        cur.att.cbn = pvacur.att.cbn
        cur.att.euler = pvacur.att.euler

        cur.att.qbn = qnn * pvapre.att.qbn * qbb
        cur.att.cbn = Rotation.quaternion2matrix(cur.att.qbn)
        cur.att.euler = Rotation.matrix2euler(cur.att.cbn)
        return cur
