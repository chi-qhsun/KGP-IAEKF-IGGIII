import numpy as np
from typing import TypeVar
from typing_extensions import Literal

T = TypeVar('T')

class Logging:
    @staticmethod
    def initialization(argv: str, logtostderr: bool = True, logtofile: bool = False):
        if (logtostderr and logtofile):
            FLAGS_alsologtostderr = True
        elif (logtostderr):
            FLAGS_logtostderr = True

        if (logtostderr):
            # // 输出颜色
            FLAGS_colorlogtostderr = True


        # // glog初始化
        google.InitGoogleLogging(argv[0])

    @staticmethod
    def printMatrix(matrix: np.ndarray, prefix: str = "Matrix: "):
        print(f"{prefix} {matrix.shape[0]}x{matrix.shape[1]}")
        if matrix.shape[1] == 1:
            print(matrix.transpose())
        else:
            print(matrix)

    @staticmethod
    def doubleData(data: float):
        return f"{data:.6f}"

