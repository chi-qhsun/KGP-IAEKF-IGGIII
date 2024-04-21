import numpy as np
from common.types import *
from .fileloader import *



class ImuFileLoader(FileLoader):
    def __init__(self, filename=None, columns=None, rate=200.0):
        super().__init__(filename,columns,FileLoader.TEXT)
        self.dt_ = 1.0 / rate
        self.imu_ = IMU()
        self.imu_pre_ = IMU()

    # iterate IMU data
    def next(self):

        self.imu_pre_.time = self.imu_.time
        self.imu_pre_.dt = self.imu_.dt
        self.imu_pre_.dtheta = self.imu_.dtheta
        self.imu_pre_.dvel = self.imu_.dvel
        self.imu_pre_.odovel = self.imu_.odovel

        self.data_ = self.load()

        self.imu_.time = self.data_[0]
        self.imu_.dtheta = np.array(self.data_[1:4])
        self.imu_.dvel = np.array(self.data_[4:7])

        dt = self.imu_.time - self.imu_pre_.time
        if dt < 0.1:
            self.imu_.dt = dt
        else:
            self.imu_.dt = self.dt_

        # Handle optional velocity data
        if self.columns_ > 7:
            self.imu_.odovel = self.data_[7] * self.imu_.dt

        return self.imu_

    def starttime(self):
        initial_pos = self.filefp_.tell()
        self.filefp_.seek(0)
        start_time = float(self.filefp_.readline().split(' ')[0])
        self.filefp_.seek(initial_pos)
        return start_time

    def endtime(self):
        endtime = -1
        initial_pos = self.filefp_.tell()

        self.filefp_.seek(0, 2)  # Move to end of file
        self.filefp_.seek(self.filefp_.tell() - 100, 0)  # Go back 100 bytes to ensure in data
        lines = self.filefp_.readlines()
        last_line = lines[-1].strip()

        end_time = float(last_line.split(' ')[0])
        self.filefp_.seek(initial_pos)
        return end_time
