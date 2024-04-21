from .types import *
import numpy as np
import math
from pyquaternion import Quaternion

class Earth:

    # earth constants
    WGS84_WIE = 7.2921151467E-5
    WGS84_F = 0.0033528106647474805
    WGS84_RA = 6378137.0000000000
    WGS84_RB = 6356752.3142451793
    WGS84_GM0 = 398600441800000.00
    WGS84_E1 = 0.0066943799901413156
    WGS84_E2 = 0.0067394967422764341

    # gravity calculation
    @staticmethod
    def gravity(blh: np.ndarray):
        sin2 = np.sin(blh[0])
        sin2 *= sin2
        return 9.7803267715 * (1 + 0.0052790414 * sin2 + 0.0000232718 * sin2 * sin2) + blh[2] * (0.0000000043977311 * sin2 - 0.0000030876910891) + 0.0000000000007211 * blh[2] * blh[2]

    # meridional radius and the prime vertical radius calculation
    @staticmethod
    def meridianPrimeVerticalRadius(lat: float):
        tmp = np.sin(lat)
        tmp *= tmp
        tmp = 1 - Earth.WGS84_E1 * tmp
        sqrttmp = np.sqrt(tmp)
        return np.array([Earth.WGS84_RA * (1 - Earth.WGS84_E1) / (sqrttmp * tmp), Earth.WGS84_RA / sqrttmp])

    # prime vertical radius calculation
    @staticmethod
    def RN(lat: float):
        sinlat = np.sin(lat)
        return Earth.WGS84_RA / np.sqrt(1.0 - Earth.WGS84_E1 * sinlat * sinlat)

    # transformation matrix from the NED (Navigation) frame to the ECEF (Earth-Centered, Earth-Fixed) frame
    @staticmethod
    def cne(blh: np.ndarray):
        sinlat = np.sin(blh[0])
        sinlon = np.sin(blh[1])
        coslat = np.cos(blh[0])
        coslon = np.cos(blh[1])

        dcm = np.zeros((3,3))
        dcm[0, 0] = -sinlat * coslon
        dcm[0, 1] = -sinlon
        dcm[0, 2] = -coslat * coslon

        dcm[1, 0] = -sinlat * sinlon
        dcm[1, 1] = coslon
        dcm[1, 2] = -coslat * sinlon

        dcm[2, 0] = coslat
        dcm[2, 1] = 0
        dcm[2, 2] = -sinlat

        return dcm

    # quaternion for transforming from the NED frame to the ECEF frame
    @staticmethod
    def qne(blh: np.ndarray):
        coslon = np.cos(blh[1] * 0.5)
        sinlon = np.sin(blh[1] * 0.5)
        coslat = np.cos(-math.pi * 0.25 - blh[0] * 0.5)
        sinlat = np.sin(-math.pi * 0.25 - blh[0] * 0.5)

        w = coslat * coslon
        x = -sinlat * sinlon
        y = sinlat * coslon
        z = coslat * sinlon

        quat = Quaternion(w=w, x=x, y=y, z=z)
        return quat

    # NED frame to the ECEF frame quaternion --> latitude and longitude
    @staticmethod
    def blh(qne: np.ndarray, height: float):
        return np.array([-2 * np.arctan(qne[2] / qne[0]) - math.pi * 0.5, 2 * np.arctan2(qne[3], qne[0]), height])  # 需要检查

    # conversion from geodetic coordinates (latitude, longitude, and altitude) to ECEF coordinates.
    @staticmethod
    def blh2ecef(blh: np.ndarray):
        coslat = np.cos(blh[0])
        sinlat = np.sin(blh[0])
        coslon = np.cos(blh[1])
        sinlon = np.sin(blh[1])

        rn = Earth.RN(blh[0])
        rnh = rn + blh[2]

        return np.array([rnh * coslat * coslon, rnh * coslat * sinlon, (rnh - rn * Earth.WGS84_E1) * sinlat])

    # conversion from ECEF to geodetic coordinates
    @staticmethod
    def ecef2blh(ecef: np.ndarray):
        p = np.sqrt(ecef[0] * ecef[0] + ecef[1] * ecef[1])
        h = 0

        # 初始状态
        lat = np.arctan(ecef[2] / (p * (1.0 - Earth.WGS84_E1)))
        lon = 2.0 * np.arctan2(ecef[1], ecef[0] + p)

        while True:
            h2 = h
            rn = Earth.RN(lat)
            h = p / np.cos(lat) - rn
            lat = np.arctan(ecef[2] / (p * (1.0 - Earth.WGS84_E1 * rn / (rn + h))))
            # Check if the difference between h and h2 is less than 1.0e-4
            if abs(h - h2) <= 1.0e-4:
                break  # Exit the loop if the condition is met

        return np.array([lat, lon, h])

    # conversion from relative positions in NED frame to relative positions in geodetic coordinates
    @staticmethod
    def DRi(blh: np.ndarray):
        dri = np.zeros((3,3))

        rmn = Earth.meridianPrimeVerticalRadius(blh[0])

        dri[0, 0] = 1.0 / (rmn[0] + blh[2])
        dri[1, 1] = 1.0 / ((rmn[1] + blh[2]) * np.cos(blh[0]))
        dri[2, 2] = -1
        return dri

    # conversion from relative positions in geodetic coordinates to relative positions in NED frame
    @staticmethod
    def DR(blh: np.ndarray):
        dr = np.zeros((3,3))

        rmn = Earth.meridianPrimeVerticalRadius(blh[0])

        dr[0, 0] = rmn[0] + blh[2]
        dr[1, 1] = (rmn[1] + blh[2]) * np.cos(blh[0])
        dr[2, 2] = -1
        return dr

    # conversion from local coordinates (unfolded at the origin) to global coordinates
    @staticmethod
    def local2global(origin: np.ndarray, local: np.ndarray):
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)

        ecef1 = ecef0 + cn0e * local
        blh1 = Earth.ecef2blh(ecef1)

        return blh1

    # conversion from global coordinates to local coordinates (unfolded at the origin)
    @staticmethod
    def global2local(origin: np.ndarray, Global: np.ndarray):
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)

        ecef1 = Earth.blh2ecef(Global)

        return cn0e.transpose() * (ecef1 - ecef0)

    @staticmethod
    def local2global(origin: np.ndarray, local: Pose):
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)

        ecef1 = ecef0 + cn0e * local.t
        blh1 = Earth.ecef2blh(ecef1)
        cn1e = Earth.cne(blh1)

        Global = Pose(cn1e.transpose() * cn0e * local.R, blh1)

        return Global

    @staticmethod
    def global2local(origin: np.ndarray, Global: Pose):
        ecef0 = Earth.blh2ecef(origin)
        cn0e = Earth.cne(origin)

        ecef1 = Earth.blh2ecef(Global.t)
        cn1e = Earth.cne(Global.t)

        local = Pose(cn0e.transpose() * cn1e * Global.R, cn0e.transpose() * (ecef1 - ecef0))

        return local

    # projection of the angular velocity of Earth rotation onto the ECEF frame
    @staticmethod
    def iewe():
        return np.array([0, 0, Earth.WGS84_WIE])

    # projection of the angular velocity of Earth rotation onto the NED frame
    @staticmethod
    def iewn(lat: float):
        return np.array([Earth.WGS84_WIE * np.cos(lat), 0, -Earth.WGS84_WIE * np.sin(lat)])

    @staticmethod
    def iewn(origin: np.ndarray, local: np.ndarray):
        Global = Earth.local2global(origin, local)
        return Earth.iewn(Global[0])

    # projection of angular velocity of NED frame relative to the ECEF frame onto the NED frame.
    @staticmethod
    def enwn(rmn_or_origin: np.ndarray, blh_or_local: np.ndarray, vel_or_vel: np.ndarray):
        if len(rmn_or_origin) == 2:  # Assumes rmn is provided
            rmn = rmn_or_origin
            blh = blh_or_local
            vel = vel_or_vel

        elif len(rmn_or_origin) == 3:  # Assumes origin is provided
            origin = rmn_or_origin
            local = blh_or_local
            vel = vel_or_vel

            Global = Earth.local2global(origin, local)
            rmn = Earth.meridianPrimeVerticalRadius(Global[0])

            blh = Global

        return np.array([
            vel[1] / (rmn[1] + blh[2]),
            -vel[0] / (rmn[0] + blh[2]),
            -vel[1] * np.tan(blh[0]) / (rmn[1] + blh[2])
        ])

