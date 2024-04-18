import os
from .filebase import *


class FileLoader(FileBase):

    def __init__(self, filename=None, columns=None, filetype=FileBase.TEXT):
        super().__init__()
        # print(self.filefp_,'',self.TEXT,'',self.filetype_)
        self.data_ = None
        self.open(filename, columns, filetype)

    def open(self, filename, columns, filetype):
        mode = 'r' if filetype == FileBase.TEXT else 'rb'
        try:
            self.filefp_ = open(filename, mode)
            self.columns_ = columns
            self.filetype_ = filetype
            self.filename = filename
            return self.isOpen()
        except IOError or FileNotFoundError:
            print("File could not be opened")
            return False

    def load(self):
        self.load_()
        return self.data_

    def loadn(self, epochs):
        for _ in range(epochs):
            if self.load_():
                self.data_.append(self.data_)
            else:
                break
        return self.data_

    def load_single(self):
        return self.load_()

    def load_multiple(self, epochs):
        for _ in range(epochs):
            if self.load_():
                self.data_.append(self.data_)
            else:
                return self.data_
        return True


    def load_(self):
        if self.isEof():
            return False

        if self.filetype_ == FileBase.TEXT:
            line = self.filefp_.readline()
            if not line:
                return False

            # Skipping whitespace and splitting by any combination of comma and whitespace
            splits = [x.strip() for x in line.split(" ") if x.strip()]

            self.data_ = [float(val) for val in splits]
            if len(self.data_) != self.columns_:
                raise ValueError("Number of columns does not match data")

        else:
            # import struct
            # data_bytes = self.filefp_.read(self.columns_ * 8)  # assuming double precision
            # if not data_bytes:
            #     return None
            # self.data_ = struct.unpack('d' * self.columns_, data_bytes)
            pass

        return True

# Example usage
if __name__ == "__main__":
    loader = FileLoader('dataset/GNSS-RTK.txt', 7, FileBase.TEXT)
    print(loader.load())
    loader.close()


