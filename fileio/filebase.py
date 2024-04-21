import os


class FileBase:
    TEXT = 0
    BINARY = 1

    def __init__(self):
        self.filefp_ = None
        self.filetype_ = self.TEXT
        self.columns_ = 0

    # close file
    def close(self):
        if self.filefp_ and not self.filefp_.closed:
            self.filefp_.close()
            self.filefp_ = None

    # check if open
    def isOpen(self):
        return self.filefp_ is not None and not self.filefp_.closed

    # check end of lines
    def isEof(self):
        if self.filefp_ and not self.filefp_.closed:
            return self.filefp_.tell() == os.fstat(self.filefp_.fileno()).st_size
        return True

    # return file - original name kept to keep integrity of porting
    def fstream(self):
        return self.filefp_

    # return column number
    def columns(self):
        return self.columns_
