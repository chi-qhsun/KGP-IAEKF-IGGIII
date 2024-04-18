import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion

class Rotation:
    @staticmethod
    def matrix2quaternion(matrix: np.ndarray):
        return Quaternion(matrix=matrix)

    @staticmethod
    def quaternion2matrix(quaternion: Quaternion):
        return quaternion.rotation_matrix

    @staticmethod
    def matrix2euler(dcm: np.ndarray):
        euler = np.zeros(3)

        euler[1] = np.arctan(-dcm[2, 0]/ (np.sqrt(dcm[2, 1] ** 2 + dcm[2, 2] ** 2) ))

        if dcm[2, 0] <= -0.999:
            euler[0] = 0
            euler[2] = np.arctan2((dcm[1, 2] - dcm[0, 1]), (dcm[0, 2] + dcm[1, 1]))
            print("[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!")
        elif dcm[2, 0] >= 0.999:
            euler[0] = 0
            euler[2] = np.pi + np.arctan2((dcm[1, 2] + dcm[0, 1]), (dcm[0, 2] - dcm[1, 1]))
            print("[WARNING] Rotation::matrix2euler: Singular Euler Angle! Set the roll angle to 0!")
        else:
            euler[0] = np.arctan2(dcm[2, 1], dcm[2, 2])
            euler[2] = np.arctan2(dcm[1, 0], dcm[0, 0])

        # Ensure heading is in range [0, 2*pi]
        euler[2] = euler[2] + 2 * np.pi if euler[2] < 0 else euler[2]

        return euler

    @staticmethod
    def quaternion2euler(quaternion: Quaternion):
        return Rotation.matrix2euler(quaternion.rotation_matrix())

    @staticmethod
    def rotvec2quaternion(rotvec: np.ndarray):
        angle = np.linalg.norm(rotvec)
        vec = rotvec / angle
        return Quaternion(axis=vec,angle=angle)

    @staticmethod
    def quaternion2vector(quaternion: Quaternion):
        return quaternion.angle*quaternion.axis

    @staticmethod
    def euler2matrix(euler: np.ndarray):
        return R.from_euler('zyx', euler).as_matrix()

    @staticmethod
    def euler2quaternion(euler: np.ndarray):
        return Rotation.matrix2quaternion(Rotation.euler2matrix(euler)) # .squeeze()

    @staticmethod
    def skewSymmetric(vector: np.ndarray):
        return np.array([[0, -vector[2], vector[1]],
                         [vector[2], 0, -vector[0]],
                         [-vector[1], vector[0], 0]])

    @staticmethod
    def quaternionleft(q: Quaternion):
        ans = np.zeros((4, 4))
        ans[0, 0] = q.w
        ans[0, 1:] = -q.vector
        ans[1:, 0] = q.vector
        ans[1:, 1:] = q.w * np.eye(3) + Rotation.skewSymmetric(q.vector)
        return ans

    @staticmethod
    def quaternionright(p: Quaternion):
        ans = np.zeros((4, 4))
        ans[0, 0] = p.w
        ans[0, 1:] = -p.vector
        ans[1:, 0] = p.vector
        ans[1:, 1:] = p.w * np.eye(3) - Rotation.skewSymmetric(p.vector)
        return ans
