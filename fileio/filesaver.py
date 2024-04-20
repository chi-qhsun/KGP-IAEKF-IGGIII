# import csv
# import os
# from .filebase import *
# from typing import List
#
# class FileSaver(FileBase):
#     # TEXT = 0
#
#     def __init__(self, filename: str = "", columns: int = 0, filetype: int = FileBase.TEXT):
#         super().__init__()
#         if filename:
#             self.open(filename, columns, filetype)
#
#     def open(self, filename: str, columns: int, filetype: int = FileBase.TEXT):
#         self.columns_ = columns
#         self.filetype_ = filetype
#         if filetype == self.TEXT:
#             self.filefp_ = open(filename, mode='w', newline='')
#             self.writer = csv.writer(self.filefp_)
#         return True
#
#     def dump(self, data: List[float]):
#         if len(data) != self.columns:
#             raise ValueError("Data length does not match number of columns")
#         self.writer.writerow(data)
#
#     def dumpn(self, data: List[List[float]]):
#         for row in data:
#             self.dump(row)
#
#     def close(self):
#         if self.filefp_:
#             self.filefp_.close()
#
#

import csv
import os
from .filebase import *
from typing import List

class FileSaver(FileBase):
    TEXT = 0

    def __init__(self, filename: str = "", columns: int = 0, filetype: int = TEXT):
        super().__init__()
        if filename:
            self.open(filename, columns, filetype)

    def open(self, filename: str, columns: int, filetype: int = TEXT):
        self.columns = columns
        self.filetype = filetype
        if filetype == self.TEXT:
            self.file = open(filename, mode='w', newline='')
            self.writer = csv.writer(self.file)
        return True

    def dump(self, data: List[float]):
        if len(data) != self.columns:
            raise ValueError("Data length does not match number of columns")
        self.writer.writerow(data)

    def dumpn(self, data: List[List[float]]):
        for row in data:
            self.dump(row)

    def close(self):
        if self.file:
            self.file.close()

