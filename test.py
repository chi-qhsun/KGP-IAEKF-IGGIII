from common.rotation import *
import numpy as np
from pyquaternion import Quaternion

quat = Rotation.rotvec2quaternion(np.array([1,0,1]))
print(quat)
print(Rotation.quaternion2vector(quat))

quat = Quaternion(w=1, x=2, y=3, z=4)
print(Rotation.quaternionleft(quat))