from .filebase import *


class FileLoader(FileBase):

    def __init__(self, filename=None, columns=None, filetype=FileBase.TEXT):
        super().__init__()
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

    # load file
    def load(self):
        self.load_()
        return self.data_

    # load multiple cycles
    def loadn(self, epochs):
        for _ in range(epochs):
            if self.load_():
                self.data_.append(self.data_)
            else:
                break
        return self.data_

    # load file
    def load_(self):
        if self.isEof():
            return False

        if self.filetype_ == FileBase.TEXT:
            line = self.filefp_.readline()
            if not line:
                return False

            # split by whitespace
            splits = [x.strip() for x in line.split(" ") if x.strip()]

            self.data_ = [float(val) for val in splits]
            if len(self.data_) != self.columns_:
                raise ValueError("Number of columns does not match data")

        else:
            # don't care about none txt
            pass

        return True



