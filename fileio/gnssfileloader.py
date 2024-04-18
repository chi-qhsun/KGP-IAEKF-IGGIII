#include "common/types.h"
#include "fileloader.h"

from common.angle import *
from common.types import *
from .fileloader import *
import numpy as np


class GnssFileLoader(FileLoader):


    def __init__(self, filename=None, columns=7):
        super().__init__(filename, columns, FileLoader.TEXT)
        self.gnss_ = GNSS()

    def next(self):
        self.data_ = self.load()

        self.gnss_.time = self.data_[0]
        self.gnss_.blh = np.array(self.data_[1:4])

        if len(self.data_) == 7:
            self.gnss_.std = np.array(self.data_[4:7])
        else:
            self.gnss_.std = np.array(self.data_[7:10])

        self.gnss_.blh[0] *= Angle.D2R  # Convert latitude from degrees to radians
        self.gnss_.blh[1] *= Angle.D2R  # Convert longitude from degrees to radians

        return self.gnss_

# Usage
# if __name__ == "__main__":
    # gnss_loader = GnssFileLoader('dataset/GNSS-RTK.txt', 7)
    # gnss_data = gnss_loader.next()
    # print(gnss_data.blh, gnss_data.std, gnss_data.time)
